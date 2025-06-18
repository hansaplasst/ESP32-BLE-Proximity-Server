#include "ble_server.h"

#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <LittleFS.h>
#include <dprintf.h>
#include <mbedtls/sha256.h>

BLEServer* pServer = nullptr;
BLECharacteristic* doorChar = nullptr;  // Characteristic (R/W) to control the door or get the status
BLECharacteristic* doorStatusChar = nullptr;
BLECharacteristic* rssiChar = nullptr;  // Characteristic (R) to read RSSI

deviceInfo* pDevInfo = nullptr;
bool triggerUpdateRSSIandSaveToJSON = false;  // Save RSSI to LittleFS
BLEAdvertising* pAdvertising = nullptr;
bool switchState = false;

// TODO: Verwijder hier. Deze functionaliteit moet in Shazam!
const char* fAutDevs = "/authorized_devices.json";  // File to store authorized devices

// Returns a human-readable name for a characteristic UUID string
const char* getCharName(const char* uuid) {
  if (strcmp(uuid, RSSI_UUID) == 0) return "RSSI";
  if (strcmp(uuid, DOOR_UUID) == 0) return "DOOR";
  if (strcmp(uuid, DOOR_STATUS_UUID) == 0) return "DOOR_STATUS";
  return uuid;  // fallback: return the UUID itself
}

/**
 * @brief Computes the SHA-256 hash of a given Bluetooth link key and returns it as a hexadecimal string.
 *
 * This function takes a Bluetooth link key (typically 16 bytes) and computes its SHA-256 hash using the mbedtls library.
 * The resulting 32-byte hash is then converted to a 64-character hexadecimal string.
 *
 * @param linkKey The Bluetooth link key to hash (expected to be of type esp_link_key, typically 16 bytes).
 * @return std::string The SHA-256 hash of the link key, represented as a 64-character hexadecimal string.
 */
std::string hashLinkKey(const esp_link_key& linkKey) {
  unsigned char result[32];                                // Voor SHA256 is de hash 32 bytes
  mbedtls_sha256(linkKey, ESP_BT_OCTET16_LEN, result, 0);  // 0 for SHA256, 1 for SHA224

  char hex[65];  // 32 bytes * 2 hex digits + 1 null terminator
  for (int i = 0; i < 32; ++i)
    sprintf(hex + i * 2, "%02x", result[i]);
  hex[64] = '\0';

  return std::string(hex);
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
    std::string macStr = BLEAddress(param->connect.remote_bda).toString();
    DPRINTF(1, "Device connected: %s", macStr.c_str());
  }

  void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
    std::string macStr = BLEAddress(param->disconnect.remote_bda).toString();
    DPRINTF(1, "Device disconnected: %s", macStr.c_str());
    if (pDevInfo) {
      delete pDevInfo;     // Free the deviceInfo object if it exists
      pDevInfo = nullptr;  // Reset the pointer to avoid dangling reference
    }
    pAdvertising->start();
  }
};

class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override {
    uint32_t currentPasskey = random(100000, 999999);
    DPRINTF(1, "Generated passkey: %06u", currentPasskey);

    // TODO: In web portal laten zien
    return currentPasskey;
  }

  void onPassKeyNotify(uint32_t passkey) override {
    DPRINTF(2, "Passkey notify: %06u", passkey);
  }

  bool onConfirmPIN(uint32_t passkey) override {
    DPRINTF(1, "Confirming passkey: %06u", passkey);
    return true;
  }

  bool onSecurityRequest() override {
    DPRINTF(1, "Security request received");
    return true;
  }

  /**
   * @brief Callback invoked upon completion of BLE authentication.
   *
   * This method is called when the BLE authentication process completes.
   *
   * On success, it marks the device as paired, updates the MAC address map, triggers saving of the authorized device,
   * and requests the device's RSSI.
   *
   * On failure, it removes the device and restarts BLE advertising after a short delay.
   *
   * @param cmpl The authentication completion structure containing the result and device address.
   */
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
    std::string macStr = BLEAddress(cmpl.bd_addr).toString();
    if (cmpl.success) {
      DPRINTF(1, "Authentication successful: %s", macStr.c_str());
      pDevInfo = getAuthorizedDeviceFromJson(hashLinkKey(cmpl.key));
      if (!pDevInfo) {
        DPRINTF(1, "Device not found in JSON, creating new deviceInfo");
        pDevInfo = new deviceInfo();
        pDevInfo->keyHash = hashLinkKey(cmpl.key);
        pDevInfo->mac = macStr;
        pDevInfo->paired = cmpl.success;
        // This will make sure the initial RSSI is added to pDevInfo and saved to the fAutDevs file on a proximity request event
        triggerUpdateRSSIandSaveToJSON = true;
      } else {
        DPRINTF(1, "Device retrieved from JSON: %s", pDevInfo->name.c_str());
        pDevInfo->mac = macStr;  // Update MAC address in case it changed
      }
      requestProximity(cmpl.bd_addr);  // Fire an RSSI request event
    } else {
      DPRINTF(2, "Authentication failed: %s", macStr.c_str());
      DPRINTF(3, "pDevInfo: %p", pDevInfo);
      // removeFromJson(pDevInfo);  // Remove pDevInfo from JSON file if authentication fails
      pAdvertising->stop();
      delay(200);             // Give BLE stack time to clean up
      pAdvertising->start();  // Restart advertising
    }
  }
};

bool notifyChar(BLECharacteristic* pChar, const char* value) {
  DPRINTF(0, "notifyChar(%s, %s)", pChar->getUUID().toString().c_str(), value);
  if (!pChar) return false;
  pChar->setValue(value);

  BLEDescriptor* cccd = pChar->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  if (!cccd) {
    DPRINTF(3, "Notify skipped: descriptor missing");
    return false;
  }

  uint8_t* val = cccd->getValue();
  bool notifyEnabled = (val != nullptr && (val[0] & 0x01));  // bit 0 = notify

  if (notifyEnabled) {
    pChar->notify();
    return true;
  } else {
    DPRINTF(0, "Notify skipped: Client not subscribed to %s", getCharName(pChar->getUUID().toString().c_str()));
    return false;
  }
}

std::string printJsonFile(File& file, const char* filename) {
  DPRINTF(0, "printJsonFile(%s)", filename);
  std::string contents;
  file.seek(0, SeekSet);
  while (file.available()) {
    contents += static_cast<char>(file.read());
  }
  file.seek(0, SeekSet);
  DPRINTF(1, "Contents of %s:\n%s", filename, contents.c_str());
  return contents;
}

class DoorCommandCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar, esp_ble_gatts_cb_param_t* param) override {
    if (!pDevInfo) {
      DPRINTF(3, "Illegal write attempt: device not registered");
      return;
    }

    if (!pDevInfo->paired) {
      DPRINTF(3, "Illegal write attempt: device not paired");
      return;
    }

    std::string value = pChar->getValue();
    DPRINTF(0, "DoorCommandCallback received: %s", value.c_str());

    if (value.length() > 0) {
      bool bState = (value == "open" || value == "close" || value == "toggle" || value == "status");
      bool bSetName = String(value.c_str()).startsWith("name=");
      bool isValidCommand = (value == "json" || value == "format" || value == "status" || bSetName || bState);

      if (!isValidCommand) {
        DPRINTF(3, "Invalid value received: '%s'", value.c_str());
        notifyChar(doorStatusChar, "Invalid command");
        return;
      }

      if (value == "open")
        switchState = true;
      if (value == "close")
        switchState = false;
      if (value == "toggle")
        switchState = !switchState;

      if (bState) {
        std::string doorStateStr = switchState ? "OPEN" : "CLOSED";
        digitalWrite(LED_BUILTIN, switchState ? HIGH : LOW);
        DPRINTF(1, "Door state: %s", doorStateStr.c_str());
        notifyChar(doorStatusChar, doorStateStr.c_str());
      }

      if (bSetName) {
        std::string newName = value.substr(5);  // Skip "name="
        if (newName.length() > 0) {
          pDevInfo->name = newName;
          addToJson(pDevInfo);  // Save updated device info to JSON
          notifyChar(doorStatusChar, ("Name set to: " + newName).c_str());
          DPRINTF(1, "Device name set to: %s", newName.c_str());
        } else {
          notifyChar(doorStatusChar, "Invalid name");
          DPRINTF(3, "Invalid name received");
        }
      }

      if (value == "json") {
        File file = LittleFS.open(fAutDevs, "r");
        if (!file) {
          DPRINTF(3, "Could not open file %s for reading", fAutDevs);
        } else {
          notifyChar(doorStatusChar, printJsonFile(file, fAutDevs).c_str());
          file.close();
        }
      }

      if (value == "format") {
        if (!LittleFS.format()) {
          DPRINTF(3, "Failed to format LittleFS\n");
        } else {
          DPRINTF(0, "LittleFS formatted successfully");
        }
      }
    }
  }
};

// TODO: Deze functionaliteit moet in Shazam!
bool getDoorState(BLEAddress peerAddress) {
  return switchState;
}

void removeFromJson(deviceInfo* device) {
  DPRINTF(0, "removeFromJson(%s)", device->keyHash.c_str());

  // Read JSON
  JsonDocument doc;
  File file = LittleFS.open(fAutDevs, "r");
  if (!file) {
    DPRINTF(3, "Could not open %s for reading", fAutDevs);
    return;
  }

  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    DPRINTF(3, "JSON parse error: %s", err.c_str());
    return;
  }

  JsonObject root = doc.as<JsonObject>();

  if (!root[device->keyHash].is<JsonObject>()) {
    DPRINTF(1, "Device %s not found in file", device->keyHash.c_str());
    return;
  }

  root.remove(device->keyHash);  // Remove device
  DPRINTF(1, "Device %s removed from JSON", device->keyHash.c_str());

  // Write to JSON
  file = LittleFS.open(fAutDevs, "w");
  if (!file) {
    DPRINTF(3, "Could not open %s for writing", fAutDevs);
    return;
  }

  serializeJsonPretty(doc, file);
  file.close();
}

void addToJson(deviceInfo* device) {
  DPRINTF(0, "addToJson(%s, %d)", device->keyHash.c_str(), device->rssi);

  // Probeer JSON in te lezen
  JsonDocument doc;
  File file = LittleFS.open(fAutDevs, "r");
  if (file) {
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
      DPRINTF(2, "JSON read error: %s. Start with empty doc.", err.c_str());
      doc.clear();
    }
  } else {
    DPRINTF(2, "File %s does not exist. Creating new one.", fAutDevs);
  }

  // Bewerken / toevoegen
  JsonObject root = doc.as<JsonObject>();
  JsonObject obj;
  if (root[device->keyHash].is<JsonObject>()) {
    DPRINTF(1, "Updating device: %s", device->name.c_str());
    obj = root[device->keyHash];
  } else {
    DPRINTF(1, "Adding new device: %s", device->name.c_str());
    obj = doc[device->keyHash].to<JsonObject>();
  }

  obj["mac"] = device->mac;
  obj["name"] = device->name;
  obj["paired"] = device->paired;
  obj["rssi"] = device->rssi;

  // Wegschrijven naar bestand
  file = LittleFS.open(fAutDevs, "w");
  if (!file) {
    DPRINTF(3, "Failed to open file %s for writing", fAutDevs);
    return;  // Return if file cannot be opened
  }

  serializeJsonPretty(doc, file);
  file.close();

  DPRINTF(1, "JSON saved to %s", fAutDevs);
}

/**
 * @brief Retrieves the authorized device information for a given keyHash.
 *
 * This function searches for a device with the specified keyHash in a JSON file
 * stored on the filesystem. If the device is found, it returns a dynamically allocated
 * deviceInfo structure containing the device's information (MAC address, paired status, RSSI, etc).
 * If the device is not found or an error occurs (e.g., file not found, JSON parsing error),
 * the function returns nullptr.
 *
 * @param keyHash The key hash of the device to search for, as a std::string.
 * @return deviceInfo* Pointer to a dynamically allocated deviceInfo structure if found,
 *         or nullptr if not found or on error. Caller is responsible for deleting the returned pointer.
 */
deviceInfo* getAuthorizedDeviceFromJson(const std::string& keyHash) {
  DPRINTF(0, "getAuthorizedDeviceFromJson(%s)", keyHash.c_str());
  File file = LittleFS.open(fAutDevs, "r");
  if (!file) {
    DPRINTF(3, "Could not open file %s for reading", fAutDevs);
    return nullptr;  // Return nullptr if file does not exist
  }

#ifdef DEBUG_LEVEL
  printJsonFile(file, fAutDevs);  // Print the JSON file content for debugging
#endif

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    DPRINTF(3, " JSON read error: %s\n", err.c_str());
    return nullptr;
  }

  JsonObject root = doc.as<JsonObject>();
  if (!root[keyHash].is<JsonObject>()) {
    DPRINTF(1, "Device not found");
    return nullptr;
  }

  JsonObject obj = root[keyHash];
  deviceInfo* dInfo = new deviceInfo();
  dInfo->keyHash = keyHash;
  dInfo->mac = obj["mac"] | dInfo->mac;           // Default to empty string if not set
  dInfo->name = obj["name"] | dInfo->name;        // Default to "unknown" if not set
  dInfo->paired = obj["paired"] | dInfo->paired;  // Default to false if not set
  dInfo->rssi = obj["rssi"] | dInfo->rssi;        // Default to -100 if not set
  DPRINTF(0, "Device found %s (%s)", dInfo->name.c_str(), dInfo->mac.c_str());
  return dInfo;
}

deviceInfo* getDeviceInfo() {
  if (pDevInfo)
    return pDevInfo;  // Return existing deviceInfo if available
  else
    return nullptr;
}

void requestProximity(BLEAddress peerAddress) {
  DPRINTF(0, "Request RSSI from: %s", peerAddress.toString().c_str());
  esp_err_t err = esp_ble_gap_read_rssi((uint8_t*)peerAddress.getNative());
  if (err != ESP_OK) {
    DPRINTF(2, "Failed (%d) to request RSSI from: %s", err, peerAddress.toString().c_str());  // LOG
  }
}

void handleGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  DPRINTF(0, "Event %d received", event);
  switch (event) {
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: {  // Handle requestProximity
      if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        std::string macStr = BLEAddress(param->read_rssi_cmpl.remote_addr).toString();

        String rssiStr = "RSSI: " + String(param->read_rssi_cmpl.rssi);
        DPRINTF(1, "RSSI %s\t%d dBm", macStr.c_str(), param->read_rssi_cmpl.rssi);

        if (pDevInfo->mac != macStr) {
          DPRINTF(3, "DeviceInfo MAC mismatch: expected %s, got %s", pDevInfo->mac.c_str(), macStr.c_str());
          return;
        }

        if (triggerUpdateRSSIandSaveToJSON) {
          DPRINTF(1, "triggerUpdateRSSIandSaveToJSON triggered");
          triggerUpdateRSSIandSaveToJSON = false;       // Reset the flag
          pDevInfo->rssi = param->read_rssi_cmpl.rssi;  // Update RSSI in pDevInfo
          addToJson(pDevInfo);                          // Save to JSON file
        }

        notifyChar(rssiChar, rssiStr.c_str());

      } else {
        DPRINTF(2, "RSSI read failed");
      }
      break;
    }
    default:
      break;
  }
}

/**
 * @brief Initializes the BLE server with the specified device name and sets up security, services, and characteristics.
 *
 * This function performs the following steps:
 * - Initializes the BLE device with the given device name.
 * - Configures BLE security settings, including encryption level, authentication mode, IO capabilities, and key size.
 * - Sets up custom GAP event handling and server callbacks.
 * - Creates a BLE service and adds the following characteristics:
 *   - RSSI characteristic for publishing the RSSI value (read/notify).
 *   - Door characteristic for receiving door commands (read/write).
 *   - Door status characteristic for notifying the current door status (read/notify).
 * - Starts the BLE service and configures advertising parameters, including service UUID, scan response, connection intervals, and device appearance.
 *
 * @param deviceName The name to assign to the BLE device.
 */
void initBLEServer(const char* deviceName) {
  DPRINTF(0, "initBLEServer");
  BLEDevice::init(deviceName);

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);  // Set encryption level to ESP_BLE_SEC_ENCRYPT
  BLEDevice::setSecurityCallbacks(new MySecurity());

  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  pSecurity->setCapability(ESP_IO_CAP_OUT);
  pSecurity->setKeySize(16);
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEDevice::setCustomGapHandler(handleGAPEvent);
  // BLEDevice::setCustomGattsHandler(gattsCallback);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Proximity char -> deze publiceert de rssi waarde
  rssiChar = pService->createCharacteristic(
      RSSI_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  rssiChar->addDescriptor(new BLE2902());

  // Deur char -> deze ontvangt commando's client open/close/toggle
  doorChar = pService->createCharacteristic(
      DOOR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  doorChar->addDescriptor(new BLE2902());  // Add BLE2902 descriptor for notifications
  doorChar->setCallbacks(new DoorCommandCallback());

  doorStatusChar = pService->createCharacteristic(
      DOOR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  doorStatusChar->addDescriptor(new BLE2902());
  notifyChar(doorStatusChar, switchState ? "OPEN" : "CLOSED");

  pService->start();

  // Start advertising the service
  pAdvertising = BLEDevice::getAdvertising();                      // BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());               // Add the service UUID to advertising
  pAdvertising->setScanResponse(true);                             // Enable scan response
  pAdvertising->setMinPreferred(0x06);                             // Set minimum preferred connection interval
  pAdvertising->setMinPreferred(0x12);                             // Set maximum preferred connection interval
  pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_REMOTE);  // BLE Remote Control Icon Appearance
  pAdvertising->start();                                           // BLEDevice::startAdvertising();

  // Checking LittleFS file system
  if (!LittleFS.begin()) {
    DPRINTF(3, "Failed to mount LittleFS\n");
    if (!LittleFS.format()) {
      DPRINTF(3, "Failed to format LittleFS\n");
    } else {
      DPRINTF(0, "LittleFS formatted successfully");
    }
  } else {
    DPRINTF(0, "LittleFS mounted successfully");
  }

  DPRINTF(0, "BLE server initialized");
}