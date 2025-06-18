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

/**
 * @class MyServerCallbacks
 * @brief Custom BLE server callbacks for handling device connection and disconnection events.
 *
 * This class inherits from BLEServerCallbacks and overrides the onConnect and onDisconnect methods
 * to provide custom behavior when a BLE client connects to or disconnects from the server.
 *
 * - onConnect: Logs the MAC address of the connecting device.
 * - onDisconnect: Logs the MAC address of the disconnecting device, frees any allocated device info,
 *   and restarts BLE advertising.
 *
 * @note
 * - Assumes existence of DPRINTF for logging, pDevInfo for device information management,
 *   and pAdvertising for BLE advertising control.
 */
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

/**
 * @class MySecurity
 * @brief Custom BLE security callbacks implementation for ESP32 BLE server.
 *
 * This class extends BLESecurityCallbacks to handle BLE security events such as passkey requests,
 * passkey notifications, PIN confirmation, security requests, and authentication completion.
 * It manages device pairing, passkey generation, and device authorization, integrating with
 * persistent storage and proximity features.
 *
 * - Generates a random 6-digit passkey for pairing.
 * - Notifies and confirms passkey during pairing process.
 * - Handles security requests and authentication completion.
 * - On successful authentication, updates or creates device authorization info and triggers RSSI update.
 * - On authentication failure, manages device removal and restarts BLE advertising.
 *
 * @note Integrates with JSON-based device authorization and proximity RSSI tracking.
 */
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

/**
 * @brief Notifies a BLE client of a characteristic value change if notifications are enabled.
 *
 * This function sets the value of the specified BLE characteristic and sends a notification
 * to connected clients if they have subscribed to notifications (i.e., if the Client Characteristic
 * Configuration Descriptor (CCCD, UUID 0x2902) has notifications enabled).
 *
 * @param pChar Pointer to the BLECharacteristic to notify.
 * @param value The new value to set for the characteristic (as a null-terminated string).
 * @return true if the notification was sent to at least one client, false otherwise (e.g., if
 *         notifications are not enabled or the descriptor is missing).
 */
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

/**
 * @brief Reads the entire contents of a JSON file and returns it as a std::string.
 *
 * This function seeks to the beginning of the provided file, reads all available bytes,
 * and appends them to a std::string. After reading, it resets the file pointer to the start.
 * It also logs debug information about the operation and the file contents.
 *
 * @param file Reference to the File object to read from. The file should be open for reading.
 * @param filename The name of the file, used for logging purposes.
 * @return std::string The contents of the file as a string.
 */
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

/**
 * @class DoorCommandCallback
 * @brief BLE characteristic callback handler for door control commands.
 *
 * This class handles write events to a BLE characteristic, interpreting and executing
 * commands related to door control and device management. Supported commands include:
 * - "open": Opens the door (sets switch state to true).
 * - "close": Closes the door (sets switch state to false).
 * - "toggle": Toggles the door state.
 * - "status": Reports the current door state ("OPEN" or "CLOSED").
 * - "name=<new_name>": Sets the device name to <new_name> and saves it to persistent storage.
 * - "json": Reads and sends the device information JSON file over BLE.
 * - "format": Formats the LittleFS filesystem.
 *
 * The callback ensures that only registered and paired devices can issue commands.
 * Invalid or unauthorized commands are rejected and reported via BLE notifications.
 *
 * @note The class assumes the existence of global or member variables/functions:
 *       - pDevInfo: Pointer to device information structure.
 *       - switchState: Boolean representing the door's current state.
 *       - doorStatusChar: BLECharacteristic used for status notifications.
 *       - notifyChar(): Function to send BLE notifications.
 *       - addToJson(): Function to update device info in JSON.
 *       - printJsonFile(): Function to read and format JSON file content.
 *       - fAutDevs: Path to the device info JSON file.
 *       - LittleFS: Filesystem object for persistent storage.
 *       - DPRINTF: Debug print macro/function.
 */
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

      if (value == "format" && pDevInfo->isAdmin) {
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

/**
 * @brief Removes a device entry from a JSON file based on its key hash.
 *
 * This function reads a JSON file containing authorized devices, removes the entry
 * corresponding to the provided device's key hash, and writes the updated JSON back to the file.
 * It logs actions and errors using DPRINTF at various verbosity levels.
 *
 * @param device Pointer to a deviceInfo structure containing the keyHash to identify the device.
 *
 * @note The function uses LittleFS for file operations and ArduinoJson for JSON parsing and serialization.
 * @note If the device is not found or file operations fail, appropriate debug messages are logged.
 */
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

/**
 * @brief Adds or updates a device entry in a JSON file stored on LittleFS.
 *
 * This function reads a JSON file containing authorized device information,
 * adds a new device or updates an existing device entry based on the provided
 * deviceInfo structure, and writes the updated JSON back to the file.
 *
 * @param device Pointer to a deviceInfo structure containing device details
 *               such as keyHash, mac address, name, paired status, and RSSI.
 *
 * The function performs the following steps:
 * 1. Attempts to read the existing JSON file from LittleFS.
 * 2. If the file does not exist or is invalid, it starts with an empty JSON document.
 * 3. Checks if the device (by keyHash) already exists in the JSON.
 *    - If it exists, updates the device information.
 *    - If not, adds a new device entry.
 * 4. Writes the updated JSON document back to the file.
 * 5. Logs actions and errors using DPRINTF.
 *
 * @note The function assumes that LittleFS is mounted and available.
 * @note The file path is specified by the global variable fAutDevs.
 */
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

/**
 * @brief Retrieves the current device information.
 *
 * This function returns a pointer to the existing deviceInfo structure if it is available.
 * If the device information is not available, it returns nullptr.
 *
 * @return deviceInfo* Pointer to the deviceInfo structure, or nullptr if not available.
 */
deviceInfo* getDeviceInfo() {
  if (pDevInfo)
    return pDevInfo;  // Return existing deviceInfo if available
  else
    return nullptr;
}

/**
 * @brief Requests the RSSI (Received Signal Strength Indicator) value from a BLE peer device.
 *
 * This function initiates a request to read the RSSI value from the specified BLE peer address.
 * It logs the request and handles any errors that may occur during the process.
 *
 * @param peerAddress The BLEAddress object representing the peer device from which to request the RSSI.
 */
void requestProximity(BLEAddress peerAddress) {
  DPRINTF(0, "Request RSSI from: %s", peerAddress.toString().c_str());
  esp_err_t err = esp_ble_gap_read_rssi((uint8_t*)peerAddress.getNative());
  if (err != ESP_OK) {
    DPRINTF(2, "Failed (%d) to request RSSI from: %s", err, peerAddress.toString().c_str());  // LOG
  }
}

/**
 * @brief Handles GAP (Generic Access Profile) BLE events, specifically processes RSSI read completion events.
 *
 * This function is called when a GAP BLE event occurs. It currently handles the ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT event,
 * which is triggered when an RSSI (Received Signal Strength Indicator) read operation completes.
 *
 * - On successful RSSI read, it:
 *   - Converts the remote device's MAC address to a string.
 *   - Logs the RSSI value and MAC address.
 *   - Checks if the MAC address matches the expected device information.
 *   - If a trigger flag is set, updates the device's RSSI, saves the information to a JSON file, and resets the flag.
 *   - Notifies the relevant BLE characteristic with the RSSI value.
 * - On failure, logs the failure.
 *
 * @param event The GAP BLE callback event type.
 * @param param Pointer to the event parameters, containing event-specific data.
 */
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