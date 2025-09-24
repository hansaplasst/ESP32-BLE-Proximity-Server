#include "BLEProximity.h"

#include <dprintf.h>
#include <mbedtls/sha256.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

bool deviceConnected = false;

/**
 * @struct SwitchMsg
 * @brief Represents a message for controlling a switch with an optional delay.
 *
 * This structure is used to specify the desired state of a switch and the delay
 * before sending a notification about the state change.
 *
 * @var SwitchMsg::target_on
 *   Indicates the desired state of the switch.
 *   - true: Switch should be OPEN.
 *   - false: Switch should be CLOSED.
 *
 * @var SwitchMsg::delay_ms
 *   The delay in milliseconds before sending a notification about the switch state change.
 */
struct SwitchMsg {
  bool target_on;     // true = OPEN, false = CLOSED
  uint32_t delay_ms;  // delay before notify
};

static QueueHandle_t sSwitchQueue = nullptr;
static TaskHandle_t sSwitchTask = nullptr;
extern BLEProximity* proximityServer;
static bool switchState = false;
static inline const char* stateToStr(bool on) { return on ? "OPEN" : "CLOSED"; }

/**
 * @brief Task function to process switch state change messages from a queue.
 *
 * This function runs in an infinite loop, waiting for messages on the `sSwitchQueue`.
 * When a message is received, it applies the specified delay (if any) and updates
 * the physical state of the switch. It also notifies connected BLE clients of the
 * new switch state using the `notifySwitch` method of the `proximityServer`.
 *
 * @param arg Pointer to task arguments (not used).
 */
static void SwitchNotifyTask(void* arg) {
  SwitchMsg msg;
  for (;;) {
    if (xQueueReceive(sSwitchQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (msg.delay_ms > 0) vTaskDelay(pdMS_TO_TICKS(msg.delay_ms));

      // The ONLY place that modifies the physical state and global switchState:
      switchState = msg.target_on;
#ifdef LED_BUILTIN
      digitalWrite(LED_BUILTIN, switchState ? HIGH : LOW);
#endif

      if (proximityServer) {
        // Send rSwitchCharacteristic update with current state
        proximityServer->notifySwitch(stateToStr(switchState));
      }
      DPRINTF(1, "Switch applied: %s", stateToStr(switchState));
    }
  }
}

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

void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (proximityServer) {
    proximityServer->handleGAPEvent(event, param);
  }
}

/**
 * @brief Constructs a BLEProximity object and initializes the BLE server and services.
 *
 * This constructor initializes the BLE device with the specified device name, sets up
 * security parameters, creates a BLE server, and defines the necessary services and
 * characteristics for proximity detection. It also starts advertising the BLE service.
 *
 * @param deviceName The name of the BLE device (default is "BLE Proximity Server").
 */
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

/**
 * @brief Initializes the BLEProximity service.
 *
 * This method performs additional initialization steps required for the BLEProximity service.
 * It creates a queue for switch messages if it does not already exist, and starts the
 * SwitchNotifyTask on core 1 (App CPU) if it is not already running.
 *
 * The method ensures that the switch queue and notification task are properly set up
 * before the BLEProximity service begins operation.
 */
void BLEProximity::begin() {
  // Additional init steps
  if (!sSwitchQueue) {
    sSwitchQueue = xQueueCreate(8, sizeof(SwitchMsg));
  }
  if (sSwitchQueue && !sSwitchTask) {
    xTaskCreatePinnedToCore(SwitchNotifyTask, "SwitchNotifyTask", 4096, nullptr, 5, &sSwitchTask, 1);  // Pin the task on core 1 (App CPU).
  }

  // ProximitySecurity::printBondedDevices();
}

/**
 * @brief Polling method for the BLEProximity service.
 *
 * This method is intended to be called periodically to perform polling tasks related
 * to the BLEProximity service. Specifically, it checks if the device is authenticated
 * and, if so, requests the RSSI (Received Signal Strength Indicator) value from the
 * connected BLE device to monitor proximity.
 */
void BLEProximity::poll() {
  // Polling tasks. Like RSSI request.
  if (device.isAuthenticated) {
    requestProximity(BLEAddress(device.data.mac));
  }
}

/**
 * @brief Sets the proximity RSSI threshold for the device.
 *
 * This method updates the `rssi_threshold` field in the `device.data` structure
 * to the specified RSSI value. The RSSI threshold is used to determine proximity
 * events based on the received signal strength indicator (RSSI) from the connected
 * BLE device.
 *
 * @param rssi The RSSI threshold value to set (in dBm).
 */
void BLEProximity::setProximityThreshold(int8_t rssi) {
  device.data.rssi_threshold = rssi;
}

/**
 * @brief Notifies connected BLE clients of the current switch state.
 *
 * This method sends notifications to connected BLE clients for both the command
 * characteristic (`rwCharacteristic`) and the switch characteristic (`rSwitchCharacteristic`)
 * with the provided switch state value. It uses the `notifyChar` helper function to
 * perform the notifications.
 *
 * @param state The current state of the switch, represented as a string ("OPEN" or "CLOSED").
 */
void BLEProximity::notifySwitch(const char* state) {
  notifyChar(rwCharacteristic, state);
  notifyChar(rSwitchCharacteristic, state);
}

/**
 * @brief Sets the switch state based on the provided command string.
 *
 * Supported commands:
 * - "open": Sets the switch to ON.
 * - "close": Sets the switch to OFF.
 * - "toggle": Toggles the current switch state.
 * - "momOpen": Momentarily sets the switch to ON, then OFF after a delay.
 * - "momClose": Momentarily sets the switch to OFF, then ON after a delay.
 * - "status": Reports the current switch state.
 *
 * Invalid commands are reported via BLE notification.
 * For momentary actions, any pending switch actions are cleared before enqueuing the new sequence.
 *
 * @param value Command string specifying the desired switch action.
 */
void BLEProximity::setSwitchState(const std::string& value) {
  if (value != "open" && value != "close" && value != "toggle" &&
      value != "momOpen" && value != "momClose" && value != "status") {
    DPRINTF(3, "Invalid command for setSwitchState: %s", value.c_str());
    notifyChar(rwCharacteristic, "Invalid command");
    return;
  }

  if (value == "status") {
    bool state = digitalRead(LED_BUILTIN) == HIGH;
    notifyChar(rwCharacteristic, stateToStr(state));
    notifyChar(rSwitchCharacteristic, stateToStr(state));
    return;
  }

  // Helper to enqueue a switch notification for rSwitchCharacteristic
  auto enqueueSwitch = [&](bool on, uint32_t delay_ms) {
    if (!sSwitchQueue) return;
    SwitchMsg m{on, delay_ms};
    xQueueSendToBack(sSwitchQueue, &m, 0);
  };

  // Handle state changes
  bool target = switchState;
  if (value == "open" || value == "momOpen")
    target = true;
  if (value == "close" || value == "momClose")
    target = false;
  if (value == "toggle")
    target = !switchState;

  // notifyChar(rwCharacteristic, stateToStr(target));

  // Non-momentary: direct notify via task
  if (value == "open" || value == "close" || value == "toggle") {
    enqueueSwitch(target, 0);
    return;
  }

  // Momentary: two-phase notifications (cancel pending first)
  if (value == "momOpen") {
    if (sSwitchQueue) xQueueReset(sSwitchQueue);  // clear old sequences
    enqueueSwitch(true, 0);
    enqueueSwitch(false, (uint32_t)device.data.momSwitchDelay);
    DPRINTF(1, "momOpen sequence: %d ms", device.data.momSwitchDelay);
  } else if (value == "momClose") {
    if (sSwitchQueue) xQueueReset(sSwitchQueue);
    enqueueSwitch(false, 0);
    enqueueSwitch(true, (uint32_t)device.data.momSwitchDelay);
    DPRINTF(1, "momClose sequence: %d ms", device.data.momSwitchDelay);
  }
}

/**
 * @brief Callback invoked when a BLE client connects to the server.
 *
 * This method handles the connection event by updating the device state and logging
 * the MAC address of the connected device. It sets the `deviceConnected` flag to true
 * and updates the `device.data.mac` field with the MAC address of the connected client.
 *
 * @param pServer Pointer to the BLEServer instance that received the connection.
 * @param param Pointer to the parameters associated with the connection event.
 */
void BLEProximity::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  DPRINTF(0, "onConnect");
  device.data.mac = BLEAddress(param->connect.remote_bda).toString();
  DPRINTF(1, "Device connected: %s\n", device.data.mac.c_str());
  deviceConnected = true;
}

/**
 * @brief Callback invoked when a BLE client disconnects from the server.
 *
 * This method handles the disconnection event by resetting the device state,
 * executing any configured disconnection commands, and restarting BLE advertising
 * to allow new connections. It also logs the disconnection event and the MAC address
 * of the disconnected device.
 *
 * @param pServer Pointer to the BLEServer instance that received the disconnection.
 * @param param Pointer to the parameters associated with the disconnection event.
 */
void BLEProximity::onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  DPRINTF(0, "onDisconnect()");
  deviceConnected = false;

  rwCharacteristic->setValue(device.data.on_disconnect_command.c_str());     // Set the command characteristic value
  if (commandCallback) commandCallback->onWrite(rwCharacteristic, nullptr);  // Call onWrite to handle the command
  delay(500);

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
          DPRINTF(0, "RSSI %s\t%d dBm", macStr.c_str(), param->read_rssi_cmpl.rssi);

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

          if (device.data.rssi_command != "" && param->read_rssi_cmpl.rssi >= device.data.rssi_threshold &&
              (millis() - device.rssiExecutedTimeStamp >= (uint32_t)device.data.rssi_command_delay * 1000)) {
            DPRINTF(1, "Measured RSSI %d ≥ %d.\n Executing RSSI command: %s",
                    param->read_rssi_cmpl.rssi, device.data.rssi_threshold, device.data.rssi_command.c_str());
            rwCharacteristic->setValue(device.data.rssi_command.c_str());              // Set the command characteristic value
            if (commandCallback) commandCallback->onWrite(rwCharacteristic, nullptr);  // Call onWrite to handle the command
            device.rssiExecutedTimeStamp = millis();
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

/**
 * @brief Callback invoked when the authentication process is completed.
 *
 * This method handles the completion of the authentication process with a BLE peer device.
 * It checks if the authentication was successful and updates the device state accordingly.
 * If the device is authenticated, it retrieves or creates the device entry in the JSON storage,
 * updates its MAC address if necessary, and triggers an RSSI request.
 * If authentication fails, it restarts BLE advertising to allow new connections.
 *
 * @param cmpl The authentication completion parameters containing status and peer address.
 */
void ProximitySecurity::onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
  DPRINTF(0, "onAuthenticationComplete: %s", cmpl.success ? "Success" : "Failure");
  device.isAuthenticated = cmpl.success;
  std::string macStr = BLEAddress(cmpl.bd_addr).toString();

  if (device.data.isBlocked) return;  // Ignore blocked devices

  if (device.isAuthenticated) {
    DPRINTF(1, "Authentication successful: %s", macStr.c_str());
    // printBondedDevices();
    std::string hashedKey = getHashedPeerKey(cmpl.bd_addr);
    if (hashedKey == "") {
      DPRINTF(2, "No bonded key found for device: %s", macStr.c_str());
      return;
    }
    // TODO: do not log hashed key in production for security reasons
    // DPRINTF(0, "Mac %s Peer key: %s", macStr.c_str(), hashedKey.c_str());
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
              "    device.data.deviceID:\t***************\n"
              "    device.data.name:\t\t%s\n"
              "    device.data.mac:\t\t%s\n"
              "    device.data.paired:\t\t%s\n"
              "    device.data.isBlocked:\t%s\n"
              "    device.data.isAdmin:\t%s\n"
              "    device.data.rssi_threshold:\t%d\n"
              "    device.data.momSwitchDelay:\t%d\n"
              "    device.data.rssi_command:\t%s\n"
              "    device.data.rssi_command_delay:\t%d\n"
              "    device.data.on_disconnect_command:\t%s",
              device.data.name.c_str(),
              device.data.mac.c_str(),
              device.data.paired ? "true" : "false",
              device.data.isBlocked ? "true" : "false",
              device.data.isAdmin ? "true" : "false",
              device.data.rssi_threshold,
              device.data.momSwitchDelay,
              device.data.rssi_command.c_str(),
              device.data.rssi_command_delay,
              device.data.on_disconnect_command.c_str());
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

/**
 * @brief Converts a 16-byte octet array to its hexadecimal string representation.
 *
 * This function takes a 16-byte octet array (esp_bt_octet16_t) and converts it to a
 * hexadecimal string representation. Each byte is represented by two hexadecimal characters.
 *
 * @param value The 16-byte octet array to convert.
 * @return std::string The hexadecimal string representation of the octet array.
 */
std::string ProximitySecurity::getHexString(const esp_bt_octet16_t& value) {
  char hex[33];  // 16 bytes * 2 hex digits + 1 null terminator

  for (size_t i = 0; i < ESP_BT_OCTET16_LEN; ++i) {
    sprintf(hex + i * 2, "%02x", value[i]);
  }
  hex[32] = '\0';

  return std::string(hex);
}

/**
 * @brief Retrieves the hashed peer key (IRK) for a bonded BLE device based on its MAC address.
 *
 * This method looks up the bonded key information for the specified MAC address and computes
 * the SHA-256 hash of the Identity Resolving Key (IRK) associated with that device. The hashed
 * IRK is returned as a hexadecimal string. If no bonded key is found, an empty string is returned.
 *
 * @param mac The MAC address of the bonded device (of type esp_bd_addr_t).
 * @return std::string The SHA-256 hash of the device's IRK as a hexadecimal string, or an empty string if not found.
 */
std::string ProximitySecurity::getHashedPeerKey(esp_bd_addr_t mac) {
  DPRINTF(0, "ProximitySecurity::getHashedPeerKey(%s)", BLEAddress(mac).toString().c_str());
  const esp_ble_bond_key_info_t* bondKey = getBondedKey(mac);
  if (!bondKey) {
    return "";
  }

  // DPRINTF(0, "Peer Key: %s", getHexString(bondKey->pid_key.irk).c_str());
  std::string irkStr = keyHash(bondKey->pid_key.irk);

  free((void*)bondKey);  // free the copy we allocated in getBondedKey()
  return irkStr;
}

/**
 * @brief Retrieves the bonded key information for a specific BLE device based on its MAC address.
 *
 * This method searches through the list of bonded devices to find the one that matches the provided MAC address.
 * If a match is found, it returns a pointer to a copy of the bonded key information. The caller is responsible
 * for freeing the returned pointer.
 *
 * @param mac The MAC address of the bonded device to search for (of type esp_bd_addr_t).
 * @return const esp_ble_bond_key_info_t* Pointer to a copy of the bonded key information if found, nullptr otherwise.
 *                                        The caller must free this pointer when done.
 */
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

  // Copy pointer to heap buffer to stack-struct
  esp_ble_bond_key_info_t* keyCopy = nullptr;
  if (result) {
    keyCopy = (esp_ble_bond_key_info_t*)malloc(sizeof(esp_ble_bond_key_info_t));
    if (keyCopy) {
      memcpy(keyCopy, result, sizeof(esp_ble_bond_key_info_t));
    }
  }

  free(bondedDevices);
  return keyCopy;  // Don't forget to free this later by the caller!
}

/**
 * @brief Prints the list of bonded BLE devices to the debug output.
 *
 * This method retrieves the list of bonded devices using the ESP-IDF function `esp_ble_get_bond_device_list`
 * and prints their MAC addresses to the debug output. It is useful for debugging and verifying
 * which devices are currently bonded with the BLE server.
 */
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

/**
 * @brief Removes all bonded devices from the BLE bond list.
 *
 * This method retrieves the list of currently bonded devices and removes each one
 * using the ESP-IDF function `esp_ble_remove_bond_device`. It logs the MAC addresses
 * of the removed devices for reference.
 */
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
 * @brief Removes a bonded device from the BLE bond list based on its MAC address.
 *
 * This method takes a MAC address as input and removes the corresponding bonded device
 * from the BLE bond list using the ESP-IDF function `esp_ble_remove_bond_device`.
 *
 * @param mac The MAC address of the bonded device to be removed (of type esp_bd_addr_t).
 */
void ProximitySecurity::removeBondedDevice(esp_bd_addr_t mac) {
  DPRINTF(0, "ProximitySecurity::removeBondedDevice(%s)", BLEAddress(mac).toString().c_str());
  esp_ble_remove_bond_device(mac);
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

  if (bleProx->device.data.isBlocked) {
    DPRINTF(3, "Illegal write attempt: device is blocked");
    return;
  }

  std::string value = pChar->getValue();
  DPRINTF(0, "CommandCallback received: %s", value.c_str());

  if (value.length() > 0) {
    bool bState = (value == "open" || value == "close" || value == "toggle" ||
                   value == "momOpen" || value == "momClose" || value == "status");
    bool bSetName = String(value.c_str()).startsWith("name=");                      // device name
    bool bSetMomDelay = String(value.c_str()).startsWith("momDelay=");              // momentary switch delay
    bool bSetRssiCmd = String(value.c_str()).startsWith("rssiCmd=");                // RSSI command
    bool bSetRssiDelay = String(value.c_str()).startsWith("rssiDelay=");            // RSSI command delay
    bool bSetDisconnectCmd = String(value.c_str()).startsWith("onDisconnectCmd=");  // on disconnect command
    bleProx->device.triggerUpdateJson = (value == "rssiUpdate");                    // RSSI update on next proximity event
    bool isValidCommand = (bSetName || bState || bSetMomDelay || bSetRssiCmd || bSetRssiDelay || bSetDisconnectCmd || bleProx->device.triggerUpdateJson ||
                           value == "json" || value == "format" || value == "status");

    if (!isValidCommand) {
      DPRINTF(3, "Invalid value received: '%s'", value.c_str());
      notifyChar(rwCharacteristic, "Invalid command");
      return;
    }

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

    if (bSetMomDelay) {
      const char* s = value.c_str() + 9;  // na "momDelay="
      char* endp = nullptr;
      long ms = strtol(s, &endp, 10);

      if (endp == s) {
        notifyChar(rwCharacteristic, "Invalid momDelay value");
        DPRINTF(3, "Invalid momDel payload: %s", value.c_str());
        return;
      }

      // Clamp naar int16_t en praktische grenzen
      if (ms < 10) ms = 10;        // minimaal 10 ms om '0' te voorkomen
      if (ms > 30000) ms = 30000;  // bovengrens 30s binnen int16_t

      bleProx->device.data.momSwitchDelay = static_cast<int16_t>(ms);
      bleProx->device.update();

      char buf[48];
      snprintf(buf, sizeof(buf), "momDel=%ld", ms);
      notifyChar(rwCharacteristic, buf);
      DPRINTF(1, "momSwitchDelay set to %ld ms", ms);
    }

    if (bSetRssiCmd) {
      std::string newCmd = value.substr(8);  // Skip "rssiCmd="
      bleProx->device.data.rssi_command = newCmd;
      bleProx->device.update();
      notifyChar(rwCharacteristic, ("rssiCmd set to: " + newCmd).c_str());
      DPRINTF(1, "rssiCmd set to: %s", newCmd.c_str());
    }

    if (bSetRssiDelay) {
      const char* s = value.c_str() + 10;  // na "rssiDelay="
      char* endp = nullptr;
      long sec = strtol(s, &endp, 10);

      if (endp == s) {
        notifyChar(rwCharacteristic, "Invalid rssiDelay value");
        DPRINTF(3, "Invalid rssiDelay payload: %s", value.c_str());
        return;
      }

      // Clamp naar int16_t en praktische grenzen
      if (sec < 0) sec = 0;        // minimaal 0s
      if (sec > 3600) sec = 3600;  // bovengrens 1 uur

      bleProx->device.data.rssi_command_delay = static_cast<int16_t>(sec);
      bleProx->device.update();

      char buf[48];
      snprintf(buf, sizeof(buf), "rssiDelay=%ld", sec);
      notifyChar(rwCharacteristic, buf);
      DPRINTF(1, "rssi_command_delay set to %ld s", sec);
    }

    if (bSetDisconnectCmd) {
      std::string newCmd = value.substr(16);  // Skip "onDisconnectCmd="
      bleProx->device.data.on_disconnect_command = newCmd;
      bleProx->device.update();
      notifyChar(rwCharacteristic, ("onDisconnectCmd set to: " + newCmd).c_str());
      DPRINTF(1, "onDisconnectCmd set to: %s", newCmd.c_str());
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