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
BLECharacteristic* commandChar = nullptr;  // Characteristic (R/W) to control the switch or get the status
BLECharacteristic* switchChar = nullptr;
BLECharacteristic* rssiChar = nullptr;  // Characteristic (R) to read RSSI

deviceInfo* pDevInfo = nullptr;
bool triggerUpdateRSSIandSaveToJSON = false;  // Save RSSI to LittleFS
BLEAdvertising* pAdvertising = nullptr;
bool switchState = false;

// TODO: Verwijder hier. Deze functionaliteit moet in Shazam!
const char* fDevices = "/authorized_devices.json";  // File to store authorized devices

// Returns a human-readable name for a characteristic UUID string
const char* getCharName(const char* uuid) {
  if (strcmp(uuid, RSSI_UUID) == 0) return "RSSI";
  if (strcmp(uuid, COMMAND_UUID) == 0) return "COMMAND";
  if (strcmp(uuid, SWITCH_UUID) == 0) return "SWITCH";
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
    DPRINTF(0, "onAuthenticationComplete: %s", cmpl.success ? "Success" : "Failure");
    std::string macStr = BLEAddress(cmpl.bd_addr).toString();
    if (cmpl.success) {
      DPRINTF(1, "Authentication successful: %s", macStr.c_str());
      if (!pDevInfo) pDevInfo = getAuthorizedDeviceFromJson(hashLinkKey(cmpl.key));
      if (!pDevInfo) {
        DPRINTF(1, "Device not found in JSON, creating new deviceInfo");
        pDevInfo = new deviceInfo();
        pDevInfo->keyHash = hashLinkKey(cmpl.key);
        pDevInfo->mac = macStr;
        pDevInfo->paired = cmpl.success;
        // This will make sure the initial RSSI is added to pDevInfo and saved to the fDevices file on a proximity request event
        triggerUpdateRSSIandSaveToJSON = true;
      } else {
        DPRINTF(1, "Device retrieved from JSON: %s", pDevInfo->name.c_str());
        DPRINTF(1,
                "pDevInfo->name:\t\t%s\n"
                "    pDevInfo->mac:\t\t%s\n"
                "    pDevInfo->paired:\t\t%s\n"
                "    pDevInfo->isAdmin:\t\t%s\n"
                "    pDevInfo->rssi_threshold:\t%d\n"
                "    pDevInfo->rssi_command:\t%s\n"
                "    pDevInfo->momSwitchDelay:\t%d",
                pDevInfo->name.c_str(),
                pDevInfo->mac.c_str(),
                pDevInfo->paired ? "true" : "false",
                pDevInfo->isAdmin ? "true" : "false",
                pDevInfo->rssi_threshold,
                pDevInfo->rssi_command.c_str(),
                pDevInfo->momSwitchDelay);
        if (pDevInfo->mac != macStr) {
          DPRINTF(1, "MAC address updated: %s -> %s", pDevInfo->mac.c_str(), macStr.c_str());
          pDevInfo->mac = macStr;  // Update MAC address if it has changed
          addToJson(pDevInfo);     // Save updated device info to JSON
        }
      }
      requestProximity(cmpl.bd_addr);  // Fire an RSSI request event
    } else {
      DPRINTF(2, "Authentication failed: %s", macStr.c_str());
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
  if (!pChar) return false;
  DPRINTF(0, "notifyChar(%s, %s)", pChar->getUUID().toString().c_str(), value);

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
 * @param filename The name of the file.
 * @return std::string The contents of the file as a string.
 */
std::string printJsonFile(const char* filename) {
  DPRINTF(0, "printJsonFile(%s)", filename);

  File file = LittleFS.open(filename, "r");
  if (!file) {
    DPRINTF(3, "Could not open file %s for reading", filename);
    return std::string();  // Return empty string if file cannot be opened
  }

  std::string contents;
  file.seek(0, SeekSet);
  while (file.available()) {
    contents += static_cast<char>(file.read());
  }
  file.seek(0, SeekSet);
  file.close();
  DPRINTF(1, "Contents of %s:\n%s", filename, contents.c_str());
  return contents;
}

void setSwitchState(const std::string& value) {
  if (value != "open" && value != "close" && value != "toggle" &&
      value != "momOpen" && value != "momClose" && value != "status") {
    DPRINTF(3, "Invalid command for setSwitchState: %s", value.c_str());
    notifyChar(commandChar, "Invalid command");
    return;
  }

  if (value == "open" || value == "momOpen")
    switchState = true;
  if (value == "close" || value == "momClose")
    switchState = false;
  if (value == "toggle")
    switchState = !switchState;

  std::string switchStateStr = switchState ? "OPEN" : "CLOSED";
  digitalWrite(LED_BUILTIN, switchState ? HIGH : LOW);
  notifyChar(commandChar, switchStateStr.c_str());
  notifyChar(switchChar, switchStateStr.c_str());
  DPRINTF(1, "Switch state: %s", switchStateStr.c_str());
  if (value == "momOpen" || value == "momClose") {
    delay(pDevInfo->momSwitchDelay);  // Delay for momOpen/momClose command
    switchState = !switchState;       // Toggle the switch state after the delay
    switchStateStr = switchState ? "OPEN" : "CLOSED";
    digitalWrite(LED_BUILTIN, switchState ? HIGH : LOW);
    notifyChar(commandChar, switchStateStr.c_str());
    notifyChar(switchChar, switchStateStr.c_str());
    DPRINTF(1, "Switch state: %s", switchStateStr.c_str());
  }
}

/**
 * @class CommandCallback
 * @brief Custom BLE characteristic callback for handling write requests to the command characteristic.
 *
 * This class extends BLECharacteristicCallbacks and overrides the onWrite method to handle
 * commands sent to the command characteristic. It processes commands such as "open", "close",
 * "toggle", "status", and "name=", and updates the switch state accordingly.
 *
 * - Validates device pairing status before processing commands.
 * - Notifies clients of the switch state and command results.
 * - Handles JSON file operations for device information.
 */
class CommandCallback : public BLECharacteristicCallbacks {
 public:
  /**
   * @brief Handles write requests to the command characteristic.
   *
   * This method is called when a write request is received on the command characteristic.
   * It processes the command, updates the switch state, and notifies clients of the result.
   * It also handles device name changes and JSON file operations.
   *
   * @param pChar Pointer to the BLECharacteristic that received the write request.
   * @param param Pointer to the BLE GATTS parameters containing the write data.

   * @pre NEVER CALL requestProximity() FROM THIS METHOD! IT WILL CAUSE A DEADLOCK!!!!
   */
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
    DPRINTF(0, "CommandCallback received: %s", value.c_str());

    if (value.length() > 0) {
      bool bState = (value == "open" || value == "close" || value == "toggle" ||
                     value == "momOpen" || value == "momClose" || value == "status");
      bool bSetName = String(value.c_str()).startsWith("name=");
      triggerUpdateRSSIandSaveToJSON = (value == "update_rssi");
      bool isValidCommand = (bSetName || bState || triggerUpdateRSSIandSaveToJSON ||
                             value == "json" || value == "format" || value == "status");

      if (!isValidCommand) {
        DPRINTF(3, "Invalid value received: '%s'", value.c_str());
        notifyChar(commandChar, "Invalid command");
        return;
      }
      // Update the switch state in the device info
      if (bState) setSwitchState(value);  // Update the switch state according to the command

      // Update the device name if the command is "name="
      if (bSetName) {
        std::string newName = value.substr(5);  // Skip "name="
        if (newName.length() > 0) {
          pDevInfo->name = newName;
          addToJson(pDevInfo);  // Save updated device info to JSON
          notifyChar(commandChar, ("Name set to: " + newName).c_str());
          DPRINTF(1, "Device name set to: %s", newName.c_str());
        } else {
          notifyChar(commandChar, "Invalid name");
          DPRINTF(3, "Invalid name received");
        }
      }

      // Publish the JSON file if requested
      if (value == "json") {  // && pDevInfo->isAdmin) {
        std::string jsonContent = printJsonFile(fDevices);
        notifyChar(commandChar, jsonContent.c_str());  // Do not enable this in production, it's insecure!
      }

      // Format the LittleFS if requested
      if (value == "format") {  //&& pDevInfo->isAdmin) {
        if (!LittleFS.format()) {
          notifyChar(commandChar, "Format failed");
          DPRINTF(3, "Failed to format LittleFS\n");
        } else {
          notifyChar(commandChar, "LittleFS formatted successfully");
          DPRINTF(0, "LittleFS formatted successfully");
        }
      } else if (value == "format") {
        notifyChar(commandChar, "Only admin can format");
        DPRINTF(3, "Format command denied: not an admin device");
      }
    }
  }
};

CommandCallback* commandCallback = nullptr;  // Callback for command characteristic

bool getSwitchState(BLEAddress peerAddress) {
  return switchState;
}

/**
 * @brief Removes a device entry from a JSON file stored on LittleFS.
 *
 * This function reads the JSON file containing authorized devices, removes the entry
 * corresponding to the specified device's keyHash, and writes the updated JSON back to the file.
 * If the device is not found, it logs an appropriate message.
 *
 * @param device Pointer to a deviceInfo structure containing the device's keyHash.
 *               The deviceInfo structure should contain at least the keyHash field.
 *
 * The function performs the following steps:
 * 1. Opens the JSON file for reading.
 * 2. Parses the JSON content into a JsonDocument.
 * 3. Checks if the device exists in the JSON object by its keyHash.
 * 4. If found, removes the device entry and logs success.
 * 5. Writes the updated JSON document back to the file.
 * 6. Closes the file after writing.
 */
void removeFromJson(deviceInfo* device) {
  DPRINTF(0, "removeFromJson(%s)", device->keyHash.c_str());

  // Read JSON
  JsonDocument doc;
  File file = LittleFS.open(fDevices, "r");
  if (!file) {
    DPRINTF(3, "Could not open %s for reading", fDevices);
    return;
  }

  file.seek(0);
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
  file = LittleFS.open(fDevices, "w");
  if (!file) {
    DPRINTF(3, "Could not open %s for writing", fDevices);
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
 * @note The file path is specified by the global variable fDevices.
 */
void addToJson(deviceInfo* device) {
  DPRINTF(0, "addToJson(%s, %d)", device->keyHash.c_str(), device->rssi_threshold);

  // Probeer JSON in te lezen
  JsonDocument doc;
  File file = LittleFS.open(fDevices, "r");
  if (file) {
    file.seek(0);
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
      DPRINTF(2, "JSON read error: %s. Start with empty doc.", err.c_str());
      doc.clear();
    }
  } else {
    DPRINTF(2, "File %s does not exist. Creating new one.", fDevices);
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
  obj["isAdmin"] = device->isAdmin;
  obj["rssi_threshold"] = device->rssi_threshold;
  obj["rssi_command"] = device->rssi_command;
  obj["momSwitchDelay"] = device->momSwitchDelay;

  // Wegschrijven naar bestand
  file = LittleFS.open(fDevices, "w");
  if (!file) {
    DPRINTF(3, "Failed to open file %s for writing", fDevices);
    return;  // Return if file cannot be opened
  }

  serializeJsonPretty(doc, file);
  file.close();

  DPRINTF(1, "JSON saved to %s", fDevices);
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
  File file = LittleFS.open(fDevices, "r");
  if (!file) {
    DPRINTF(3, "Could not open file %s for reading", fDevices);
    return nullptr;  // Return nullptr if file does not exist
  }

  file.seek(0);
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  DPRINTF(1, "JSON read successfully from %s", fDevices);

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
  dInfo->name = obj["name"] | dInfo->name;                                // Default to "unknown" if not set
  dInfo->mac = obj["mac"] | dInfo->mac;                                   // Default to empty string if not set
  dInfo->paired = obj["paired"] | dInfo->paired;                          // Default to false if not set
  dInfo->isAdmin = obj["isAdmin"] | dInfo->isAdmin;                       // Default to false if not set
  dInfo->rssi_threshold = obj["rssi_threshold"] | dInfo->rssi_threshold;  // Default to -100 if not set
  dInfo->rssi_command = obj["rssiCommand"] | dInfo->rssi_command;         // Default to "momOpen" if not set
  dInfo->momSwitchDelay = obj["momSwitchDelay"] | dInfo->momSwitchDelay;  // Default to 300ms if not set
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
 * @brief Handles GAP (Generic Access Profile) events related to BLE operations.
 *
 * This function processes various GAP events, specifically focusing on reading the RSSI value
 * when the ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT event is received. It updates the device's RSSI
 * threshold if requested and notifies connected clients of the new RSSI value.
 *
 * @param event The GAP event type that occurred.
 * @param param Pointer to the parameters associated with the event.
 */
void handleGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  DPRINTF(0, "Event %d received", event);
  switch (event) {
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: {  // Handle requestProximity
      if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        std::string macStr = BLEAddress(param->read_rssi_cmpl.remote_addr).toString();

        DPRINTF(1, "RSSI %s\t%d dBm", macStr.c_str(), param->read_rssi_cmpl.rssi);

        if (pDevInfo->mac != macStr) {
          DPRINTF(3, "DeviceInfo MAC mismatch: expected %s, got %s", pDevInfo->mac.c_str(), macStr.c_str());
          return;
        }

        // Check if we need to update the RSSI threshold and save to JSON
        if (triggerUpdateRSSIandSaveToJSON) {
          DPRINTF(1, "triggerUpdateRSSIandSaveToJSON triggered");
          triggerUpdateRSSIandSaveToJSON = false;  // Reset the flag
          DPRINTF(1, "Updating RSSI: %d -> %d dBm", pDevInfo->rssi_threshold, param->read_rssi_cmpl.rssi);
          pDevInfo->rssi_threshold = param->read_rssi_cmpl.rssi;  // Update RSSI in pDevInfo
          addToJson(pDevInfo);                                    // Save to JSON file
          notifyChar(commandChar, "RSSI updated");
        }

        if (pDevInfo->rssi_command != "" && param->read_rssi_cmpl.rssi >= pDevInfo->rssi_threshold) {
          DPRINTF(1, "Measured RSSI %d > %d.\n Executing RSSI command: %s",
                  param->read_rssi_cmpl.rssi, pDevInfo->rssi_threshold, pDevInfo->rssi_command.c_str());
          commandChar->setValue(pDevInfo->rssi_command.c_str());                // Set the command characteristic value
          if (commandCallback) commandCallback->onWrite(commandChar, nullptr);  // Call onWrite to handle the command
        }

        notifyChar(rssiChar, String(param->read_rssi_cmpl.rssi).c_str());

      } else {
        DPRINTF(2, "RSSI read failed");
      }
      break;
    }
    default:
      break;
  }
}

void printESPInfo() {
  DPRINTF(1, "Total heap: %d", ESP.getHeapSize());
  DPRINTF(1, "Free heap: %d", ESP.getFreeHeap());
  DPRINTF(1, "Total PSRAM: %d", ESP.getPsramSize());
  DPRINTF(1, "Free PSRAM: %d", ESP.getFreePsram());
  DPRINTF(1, "Min free heap: %d", ESP.getMinFreeHeap());
  DPRINTF(1, "Min free PSRAM: %d", ESP.getMinFreePsram());
  DPRINTF(1, "Max alloc heap: %d", ESP.getMaxAllocHeap());
  DPRINTF(1, "Max alloc PSRAM: %d", ESP.getMaxAllocPsram());
  DPRINTF(1, "Chip revision: %d", ESP.getChipRevision());
  DPRINTF(1, "Chip model: %s", ESP.getChipModel());
  DPRINTF(1, "Chip cores: %d", ESP.getChipCores());
  DPRINTF(1, "CPU frequency: %d MHz", ESP.getCpuFreqMHz());
  DPRINTF(1, "SDK version: %s", ESP.getSdkVersion());
  DPRINTF(1, "Flash chip size: %d bytes", ESP.getFlashChipSize());
  DPRINTF(1, "Flash chip speed: %d MHz", ESP.getFlashChipSpeed());
  DPRINTF(1, "Flash chip mode: %d", ESP.getFlashChipMode());
  DPRINTF(1, "Sketch size: %d bytes", ESP.getSketchSize());
  DPRINTF(1, "Sketch MD5: %s", ESP.getSketchMD5().c_str());
  DPRINTF(1, "Free sketch space: %d bytes", ESP.getFreeSketchSpace());
  DPRINTF(1, "MAC address: %016llx", ESP.getEfuseMac());
  DPRINTF(1, "Cycle count: %d", ESP.getCycleCount());
  DPRINTF(1, "Magic flash chip size: %d bytes", ESP.magicFlashChipSize(0));
  DPRINTF(1, "Magic flash chip speed: %d MHz", ESP.magicFlashChipSpeed(0));
  DPRINTF(1, "Magic flash chip mode: %d", ESP.magicFlashChipMode(0));
  DPRINTF(1, "Magic flash chip size (byte): %d", ESP.magicFlashChipSize(1));
  DPRINTF(1, "Magic flash chip speed (byte): %d", ESP.magicFlashChipSpeed(1));
  DPRINTF(1, "Magic flash chip mode (byte): %d", ESP.magicFlashChipMode(1));
  DPRINTF(1, "Magic flash chip size (byte): %d", ESP.magicFlashChipSize(2));
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
 *   - Command characteristic for receiving commands (read/write).
 *   - Switch status characteristic for notifying the current switch status (read/notify).
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

  // Proximity char -> deze publiceert de rssi_threshold waarde
  rssiChar = pService->createCharacteristic(
      RSSI_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  rssiChar->addDescriptor(new BLE2902());

  // Command char -> deze ontvangt commando's client open/close/toggle/status/name/json/format
  // Command characteristic: 6e400003-b5a3-f393-e0a
  switchChar = pService->createCharacteristic(
      SWITCH_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  switchChar->addDescriptor(new BLE2902());
  notifyChar(switchChar, switchState ? "OPEN" : "CLOSED");

  // Deur char -> deze ontvangt commando's client open/close/toggle
  // Switch control characteristic: 6e400002-b5a3-f393-e0a9-e50e24dcca9e
  commandChar = pService->createCharacteristic(
      COMMAND_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  commandChar->addDescriptor(new BLE2902());  // Add BLE2902 descriptor for notifications
  commandCallback = new CommandCallback();    // Create a new CommandCallback instance
  commandChar->setCallbacks(commandCallback);

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
  // LittleFS.format();
  DPRINTF(0, "BLE server initialized");
}