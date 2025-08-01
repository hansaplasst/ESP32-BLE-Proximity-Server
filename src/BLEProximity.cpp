#include "BLEProximity.h"

#include <dprintf.h>
#include <mbedtls/sha256.h>

bool switchState = false;
bool deviceConnected = false;

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
  bool notifyEnabled = (val != nullptr && (val[0] & 0x01) && deviceConnected);  // bit 0 = notify

  if (notifyEnabled) {
    pChar->notify();
    return true;
  } else {
    DPRINTF(0, "Notify skipped: Client not subscribed to %s", pChar->getUUID().toString().c_str());
    return false;
  }
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

BLEProximity* g_bleProximity = nullptr;
void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (g_bleProximity) {
    g_bleProximity->handleGAPEvent(event, param);
  }
}

BLEProximity::BLEProximity(const char* deviceName) {
  DPRINTF(0, "BLEProximity(%s)", deviceName);

  BLEDevice::init(deviceName);

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);  // Set encryption level to ESP_BLE_SEC_ENCRYPT
  BLEDevice::setSecurityCallbacks(new ProximitySecurity(device));

  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  pSecurity->setCapability(ESP_IO_CAP_OUT);
  pSecurity->setKeySize(16);
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  g_bleProximity = this;
  BLEDevice::setCustomGapHandler(gapEventHandler);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(this);

  pService = pServer->createService(SERVICE_UUID);  // Immediate Alert Service (Proximity)

  rwCharacteristic = pService->createCharacteristic(
      COMMAND_UUID,  // Alert Notification Control Point - Commands characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  rwCharacteristic->addDescriptor(new BLE2902());
  commandCallback = new CommandCallback(this);
  rwCharacteristic->setCallbacks(commandCallback);

  rProximityCharacteristic = pService->createCharacteristic(
      RSSI_UUID,  // Alert Level - RSSI characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  rProximityCharacteristic->addDescriptor(new BLE2902());

  rSwitchCharacteristic = pService->createCharacteristic(
      SWITCH_UUID,  // Alert Status - Proximity Switch characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  rSwitchCharacteristic->addDescriptor(new BLE2902());
  notifyChar(rSwitchCharacteristic, switchState ? "OPEN" : "CLOSED");

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();                      // BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());               // Add the service UUID to advertising
  pAdvertising->setScanResponse(true);                             // Enable scan response
  pAdvertising->setMinPreferred(0x06);                             // Set minimum preferred connection interval
  pAdvertising->setMinPreferred(0x12);                             // Set maximum preferred connection interval
  pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_REMOTE);  // BLE Remote Control Icon Appearance
  pAdvertising->start();                                           // BLEDevice::startAdvertising();

  DPRINTF(1, "BLE advertising started\n");
}

void BLEProximity::begin() {
  // Additionele init steps
  // ProximitySecurity::printBondedDevices();
}

void BLEProximity::poll() {
  // Polling taken zoals RSSI meten
  if (device.isAuthenticated) {
    requestProximity(BLEAddress(device.data.mac));
  }
}

void BLEProximity::setProximityThreshold(int8_t rssi) {
  device.data.rssi_threshold = rssi;
}

void BLEProximity::setSwitchState(const std::string& value) {
  if (value != "open" && value != "close" && value != "toggle" &&
      value != "momOpen" && value != "momClose" && value != "status") {
    DPRINTF(3, "Invalid command for setSwitchState: %s", value.c_str());
    notifyChar(rwCharacteristic, "Invalid command");
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
  notifyChar(rwCharacteristic, switchStateStr.c_str());
  notifyChar(rSwitchCharacteristic, switchStateStr.c_str());
  DPRINTF(1, "Switch state: %s", switchStateStr.c_str());
  if (value == "momOpen" || value == "momClose") {
    delay(device.data.momSwitchDelay);  // Delay for momOpen/momClose command
    switchState = !switchState;         // Toggle the switch state after the delay
    switchStateStr = switchState ? "OPEN" : "CLOSED";
    digitalWrite(LED_BUILTIN, switchState ? HIGH : LOW);
    notifyChar(rwCharacteristic, switchStateStr.c_str());
    notifyChar(rSwitchCharacteristic, switchStateStr.c_str());
    DPRINTF(1, "Switch state: %s", switchStateStr.c_str());
  }
}

void BLEProximity::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  DPRINTF(0, "onConnect");
  device.data.mac = BLEAddress(param->connect.remote_bda).toString();
  DPRINTF(1, "Device connected: %s\n", device.data.mac.c_str());
  deviceConnected = true;
}

void BLEProximity::onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  DPRINTF(0, "onDisconnect()");
  deviceConnected = false;

  rwCharacteristic->setValue(device.data.on_disconnect_command.c_str());     // Set the command characteristic value
  if (commandCallback) commandCallback->onWrite(rwCharacteristic, nullptr);  // Call onWrite to handle the command
  delay(500);
  device.isAuthenticated = false;

  DPRINTF(1, "Device disconnected (%s), advertising restarted\n", device.data.mac.c_str());
  device = {};  // Reset all fields to default
  pAdvertising->start();
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
void BLEProximity::handleGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  DPRINTF(0, "Event %d received", (int)event);
  switch (event) {
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: {  // Handle requestProximity
      if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        if (xSemaphoreTake(device.mutex, portMAX_DELAY) == pdTRUE) {
          // BEGIN CRITICAL SECTION
          std::string macStr = BLEAddress(param->read_rssi_cmpl.remote_addr).toString();
          DPRINTF(1, "RSSI %s\t%d dBm", macStr.c_str(), param->read_rssi_cmpl.rssi);

          if (device.data.mac != macStr) {
            DPRINTF(3, "ProximityDevice MAC mismatch: expected %s, got %s", device.data.mac.c_str(), macStr.c_str());
            xSemaphoreGive(device.mutex);
            break;
          }

          // Check if we need to update the RSSI threshold and save to JSON
          if (device.triggerUpdateJson) {
            DPRINTF(1, "JSON Update Triggered");
            DPRINTF(1, "Updating RSSI: %d -> %d dBm", device.data.rssi_threshold, param->read_rssi_cmpl.rssi);
            device.data.rssi_threshold = param->read_rssi_cmpl.rssi;  // Update RSSI in device
            device.update();                                          // Update the json file
            notifyChar(rwCharacteristic, "RSSI updated");
          }

          if (device.data.rssi_command != "" && param->read_rssi_cmpl.rssi >= device.data.rssi_threshold) {
            DPRINTF(1, "Measured RSSI %d ≥ %d.\n Executing RSSI command: %s",
                    param->read_rssi_cmpl.rssi, device.data.rssi_threshold, device.data.rssi_command.c_str());
            rwCharacteristic->setValue(device.data.rssi_command.c_str());              // Set the command characteristic value
            if (commandCallback) commandCallback->onWrite(rwCharacteristic, nullptr);  // Call onWrite to handle the command
          }

          notifyChar(rProximityCharacteristic, String(param->read_rssi_cmpl.rssi).c_str());
          // END CRITICAL SECTION
          xSemaphoreGive(device.mutex);
        } else {
          DPRINTF(2, "WARNING: Could not acquire mutex in handleGAPEvent()");
        }
      } else {
        DPRINTF(2, "RSSI read failed");
      }
      break;
    }
    default:
      break;
  }
}

ProximitySecurity::ProximitySecurity(ProximityDevice& devData) : device(devData) {}

uint32_t ProximitySecurity::onPassKeyRequest() {
  uint32_t currentPasskey = random(100000, 999999);
  DPRINTF(1, "Generated passkey: %06u", currentPasskey);

  // TODO: In web portal laten zien
  return currentPasskey;
}

void ProximitySecurity::onPassKeyNotify(uint32_t passkey) {
  DPRINTF(2, "Passkey notify: %06u", passkey);
}

bool ProximitySecurity::onConfirmPIN(uint32_t passkey) {
  DPRINTF(1, "Confirming passkey: %06u", passkey);
  return true;
}

bool ProximitySecurity::onSecurityRequest() {
  DPRINTF(1, "Security request received");
  return true;
}

void ProximitySecurity::onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
  DPRINTF(0, "onAuthenticationComplete: %s", cmpl.success ? "Success" : "Failure");
  device.isAuthenticated = cmpl.success;
  std::string macStr = BLEAddress(cmpl.bd_addr).toString();
  if (device.isAuthenticated) {
    DPRINTF(1, "Authentication successful: %s", macStr.c_str());
    // printBondedDevices();
    std::string hashedKey = getHashedPeerKey(cmpl.bd_addr);
    DPRINTF(1, "Mac %s Peer key: %s", macStr.c_str(), hashedKey.c_str());
    if (!device.get(hashedKey)) {
      DPRINTF(1, "Device not found in JSON, creating new device");
      device.data.deviceID = hashedKey;
      device.data.mac = macStr;
      device.data.paired = cmpl.success;

      // This will make sure the initial RSSI is added to device and saved to the device file on a proximity request event
      device.triggerUpdateJson = true;
    } else {
      DPRINTF(0,
              "Device retrieved from JSON:\n"
              "    device.data.name:\t\t%s\n"
              "    device.data.mac:\t\t%s\n"
              "    device.data.paired:\t\t%s\n"
              "    device.data.isAdmin:\t%s\n"
              "    device.data.rssi_threshold:\t%d\n"
              "    device.data.rssi_command:\t%s\n"
              "    device.data.momSwitchDelay:\t%d",
              device.data.name.c_str(),
              device.data.mac.c_str(),
              device.data.paired ? "true" : "false",
              device.data.isAdmin ? "true" : "false",
              device.data.rssi_threshold,
              device.data.rssi_command.c_str(),
              device.data.momSwitchDelay);
      if (device.data.mac != macStr) {
        DPRINTF(1, "MAC address updated: %s -> %s", device.data.mac.c_str(), macStr.c_str());
        device.data.mac = macStr;  // Update MAC address if it has changed
        device.update();           // Save updated device data to JSON
      }
    }
    requestProximity(cmpl.bd_addr);  // Fire an RSSI request event
  } else {
    DPRINTF(2, "Authentication failed: %s", macStr.c_str());

    BLEDevice::stopAdvertising();
    delay(200);                     // Give BLE stack time to clean up
    BLEDevice::startAdvertising();  // Restart advertising
  }
}

/**
 * @brief Computes the SHA-256 hash of a given Bluetooth link key and returns it as a hexadecimal string.
 *
 * This function takes a Bluetooth link key (typically 16 bytes) and computes its SHA-256 hash using the mbedtls library.
 * The resulting 32-byte hash is then converted to a 64-character hexadecimal string.
 *
 * @param linkKey The Bluetooth link key to hash (expected to be of type esp_bt_octet16_t, typically 16 bytes).
 * @return std::string The SHA-256 hash of the link key, represented as a 64-character hexadecimal string.
 */
std::string ProximitySecurity::keyHash(const esp_bt_octet16_t& key) {
  DPRINTF(0, "ProximitySecurity::keyHash()");
  unsigned char result[32];                            // For SHA256 the hash size = 32 bytes
  mbedtls_sha256(key, ESP_BT_OCTET16_LEN, result, 0);  // 0 for SHA256, 1 for SHA224

  char hex[65];  // 32 bytes * 2 hex digits + 1 null terminator
  for (int i = 0; i < 32; ++i)
    sprintf(hex + i * 2, "%02x", result[i]);
  hex[64] = '\0';

  return std::string(hex);
}

std::string ProximitySecurity::getHexString(const esp_bt_octet16_t& value) {
  char hex[33];  // 16 bytes * 2 hex digits + 1 null terminator

  for (size_t i = 0; i < ESP_BT_OCTET16_LEN; ++i) {
    sprintf(hex + i * 2, "%02x", value[i]);
  }
  hex[32] = '\0';

  return std::string(hex);
}

std::string ProximitySecurity::getHashedPeerKey(esp_bd_addr_t mac) {
  DPRINTF(0, "ProximitySecurity::getHashedPeerKey(%s)", BLEAddress(mac).toString().c_str());
  const esp_ble_bond_key_info_t* bondKey = getBondedKey(mac);
  if (!bondKey) {
    return "";
  }

  // DPRINTF(0, "Peer Key: %s", getHexString(bondKey->pid_key.irk).c_str());  // TODO: Remove. Insecure
  std::string irkStr = keyHash(bondKey->pid_key.irk);

  free((void*)bondKey);  // free the copy we allocated in getBondedKey()
  return irkStr;
}

const esp_ble_bond_key_info_t* ProximitySecurity::getBondedKey(esp_bd_addr_t mac) {
  DPRINTF(0, "ProximitySecurity::getBondedKey(%s)", BLEAddress(mac).toString().c_str());
  int pairedDevices = esp_ble_get_bond_device_num();
  if (pairedDevices <= 0) {
    DPRINTF(2, "No bonded devices found");
    return nullptr;
  }

  esp_ble_bond_dev_t* bondedDevices = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * pairedDevices);
  if (!bondedDevices) {
    DPRINTF(3, "Memory allocation for bonded devices failed");
    return nullptr;
  }

  esp_ble_get_bond_device_list(&pairedDevices, bondedDevices);

  const esp_ble_bond_key_info_t* result = nullptr;

  for (int i = 0; i < pairedDevices; i++) {
    if (memcmp(bondedDevices[i].bd_addr, mac, ESP_BD_ADDR_LEN) == 0) {
      result = &bondedDevices[i].bond_key;
      break;
    }
  }

  if (!result) {
    DPRINTF(2, "No bonded device matches the given MAC.");
  }

  // Kopieer pointer naar heap buffer naar stack-struct
  esp_ble_bond_key_info_t* keyCopy = nullptr;
  if (result) {
    keyCopy = (esp_ble_bond_key_info_t*)malloc(sizeof(esp_ble_bond_key_info_t));
    if (keyCopy) {
      memcpy(keyCopy, result, sizeof(esp_ble_bond_key_info_t));
    }
  }

  free(bondedDevices);
  return keyCopy;  // moet later worden vrijgegeven door de caller!
}

void ProximitySecurity::printBondedDevices() {
  DPRINTF(0, "ProximitySecurity::printBondedDevices()");
  int wPairedDevices = esp_ble_get_bond_device_num();
  if (wPairedDevices) {
    esp_ble_bond_dev_t* bondedDevices = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * wPairedDevices);
    if (bondedDevices) {
      DPRINTF(1, "Bonded device list:")
      esp_ble_get_bond_device_list(&wPairedDevices, bondedDevices);
      for (int i = 0; i < wPairedDevices; i++) {
        DPRINTF(1, "  MAC: %s", BLEAddress(bondedDevices[i].bd_addr).toString().c_str());
      }
      free(bondedDevices);
    }
  }
}

void ProximitySecurity::removeBondedDevices() {
  DPRINTF(0, "ProximitySecurity::removeBondedDevices()");
  int wPairedDevices = esp_ble_get_bond_device_num();
  if (wPairedDevices) {
    esp_ble_bond_dev_t* bondedDevices = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * wPairedDevices);
    if (bondedDevices) {
      DPRINTF(1, "Remove bonded devices:")
      esp_ble_get_bond_device_list(&wPairedDevices, bondedDevices);
      for (int i = 0; i < wPairedDevices; i++) {
        esp_ble_remove_bond_device(bondedDevices[i].bd_addr);
        DPRINTF(1, "  %s", BLEAddress(bondedDevices[i].bd_addr).toString().c_str());
      }
      free(bondedDevices);
    }
  }
}

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
void CommandCallback::onWrite(BLECharacteristic* pChar, esp_ble_gatts_cb_param_t* param) {
  DPRINTF(0, "onWrite()");
  if (!bleProx->device.data.paired) {
    DPRINTF(3, "Illegal write attempt: device not paired");
    return;
  }

  std::string value = pChar->getValue();
  DPRINTF(0, "CommandCallback received: %s", value.c_str());

  if (value.length() > 0) {
    bool bState = (value == "open" || value == "close" || value == "toggle" ||
                   value == "momOpen" || value == "momClose" || value == "status");
    bool bSetName = String(value.c_str()).startsWith("name=");
    bleProx->device.triggerUpdateJson = (value == "update_rssi");
    bool isValidCommand = (bSetName || bState || bleProx->device.triggerUpdateJson ||
                           value == "json" || value == "format" || value == "status");

    if (!isValidCommand) {
      DPRINTF(3, "Invalid value received: '%s'", value.c_str());
      notifyChar(rwCharacteristic, "Invalid command");
      return;
    }
    // Update the switch state in the device data
    if (bState) bleProx->setSwitchState(value);  // Update the switch state according to the command

    // Update the device name if the command is "name="
    if (bSetName) {
      std::string newName = value.substr(5);  // Skip "name="
      if (newName.length() > 0) {
        bleProx->device.data.name = newName;
        bleProx->device.update();
        notifyChar(rwCharacteristic, ("Name set to: " + newName).c_str());
        DPRINTF(1, "Device name set to: %s", newName.c_str());
      } else {
        notifyChar(rwCharacteristic, "Invalid name");
        DPRINTF(3, "Invalid name received");
      }
    }

    // Publish the JSON file if requested
    if (value == "json" && bleProx->device.data.isAdmin) {
      bleProx->device.printJsonFile();  // TODO: Do NOT enable this in production, it's insecure!
      notifyChar(rwCharacteristic, "Disabled for security reasons");
    }

    // Format the LittleFS if requested
    if (value == "format" && bleProx->device.data.isAdmin) {
      if (!LittleFS.format()) {
        notifyChar(rwCharacteristic, "Format failed");
        DPRINTF(3, "Failed to format LittleFS\n");
      } else {
        notifyChar(rwCharacteristic, "LittleFS formatted successfully");
        DPRINTF(0, "LittleFS formatted successfully");
        ProximitySecurity::removeBondedDevices();
      }
    } else if (value == "format") {
      notifyChar(rwCharacteristic, "Only admin can format");
      DPRINTF(3, "Format command denied: not an admin device");
    }
  }
}

CommandCallback::CommandCallback(BLEProximity* BLEProx) : bleProx(BLEProx) {}