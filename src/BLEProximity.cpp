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

// Callback hook handlers
ProximitySecurity::PasskeyNotifyHandler ProximitySecurity::s_passkeyHandler = nullptr;
ProximitySecurity::AuthResultHandler ProximitySecurity::s_authResultHandler = nullptr;

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
      if (proximityServer) {
        // Update the physical pin via the BLEProximity instance and send notification
        digitalWrite(proximityServer->device.getSwitchPin(), switchState ? HIGH : LOW);

        // Send switchChar update with current state
        proximityServer->notifySwitch(stateToStr(switchState));
      } else {
        DPRINTF(2, "Switch update skipped: proximityServer is null");
      }
      DPRINTF(1, "Switch: %s", stateToStr(switchState));
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

void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (proximityServer) {
    proximityServer->handleGAPEvent(event, param);
  }
}

/**
 * @brief Construct a BLEProximity instance and set ProximityDevice
 */
BLEProximity::BLEProximity(ProximityDevice& dev) : device(dev) {
  DPRINTF(0, "BLEProximity(%s)", dev.name.c_str());
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
  DPRINTF(0, "BLEProximity::begin()");
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

  DPRINTF(0, " BLE Init Start");
  BLEDevice::init(device.name.c_str());

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

  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(this);

  pService = pBLEServer->createService(SERVICE_UUID);  // Immediate Alert Service (Proximity)

  DPRINTF(0, " Init BLE Characteristics");
  cmdChar = pService->createCharacteristic(
      COMMAND_UUID,  // Alert Notification Control Point - Commands characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  cmdChar->addDescriptor(new BLE2902());
  commandCallback = new CommandCallback(this);
  cmdChar->setCallbacks(commandCallback);

  proximityChar = pService->createCharacteristic(
      RSSI_UUID,  // Alert Level - RSSI characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  proximityChar->addDescriptor(new BLE2902());

  switchChar = pService->createCharacteristic(
      SWITCH_UUID,  // Alert Status - Proximity Switch characteristic
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  switchChar->addDescriptor(new BLE2902());
  // notifyChar(switchChar, switchState ? "OPEN" : "CLOSED");

  pService->start();

  DPRINTF(0, " Start BLE advertising")
  pAdvertising = BLEDevice::getAdvertising();                      // BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());               // Add the service UUID to advertising
  pAdvertising->setScanResponse(true);                             // Enable scan response
  pAdvertising->setMinPreferred(0x06);                             // Set minimum preferred connection interval
  pAdvertising->setMinPreferred(0x12);                             // Set maximum preferred connection interval
  pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_REMOTE);  // BLE Remote Control Icon Appearance
  pAdvertising->start();                                           // BLEDevice::startAdvertising();

  delay(100);  // small delay to ensure advertising starts

  DPRINTF(0, " Pin Switch Queue thread to Core 0")
  if (!sSwitchQueue) {
    DPRINTF(0, "  Creating Queue");
    sSwitchQueue = xQueueCreate(8, sizeof(SwitchMsg));
  }
  if (sSwitchQueue && !sSwitchTask) {
    DPRINTF(0, "  Creating Task for Core 0");
    /**
     * Switch Notification Task
     * Core 0: System CPU, mostly used for system tasks
     * Core 1: App CPU, suitable for user tasks
     */
    BaseType_t res = xTaskCreatePinnedToCore(
        SwitchNotifyTask,      // task function
        "SwitchNotifyWorker",  // name
        4096,                  // stack size in words (8192*4 bytes)
        nullptr,               // parameter
        5,                     // priority
        &sSwitchTask,          // task handle
        0                      // core id (0 or 1)
    );
    if (res != pdPASS) {
      DPRINTF(3, "Failed to create SwitchNotifyWorker task");
    }
  }
  delay(50);  // small delay to ensure task is started
  // ProximitySecurity::printBondedDevices();

  DPRINTF(0, " Ensure physical pin matches initial state")
  setSwitchState(switchState ? "open" : "close");  // Ensure physical pin matches initial state

  // Create the command queue (e.g. max 10 pending commands)
  commandQueue = xQueueCreate(10, sizeof(ProximityCommand));
  if (!commandQueue) {
    DPRINTF(3, "Failed to create command queue");
  } else {
    // Create worker task on core 1 (or 0), with a decent stack size
    BaseType_t res = xTaskCreatePinnedToCore(
        BLEProximity::commandWorkerTask,  // task function
        "CommandWorker",                  // name
        8192,                             // stack size in words (8192*4 bytes)
        this,                             // parameter
        5,                                // priority
        nullptr,                          // task handle
        1                                 // core id (0 or 1)
    );
    if (res != pdPASS) {
      DPRINTF(3, "Failed to create CommandWorker task");
    }
  }

#if DEBUG_LEVEL == 0
  delay(50);  // small delay to ensure debug messages are in sync
#endif

  DPRINTF(0, " BLE Init Finished...")
  DPRINTF(1,
          "Bluetooth Low Energy Service started\n\t"
          "Open Bluetooth settings and pair with: %s",
          device.name.c_str());
}

/**
 * @brief Polls for RSSI updates from the connected BLE device.
 *
 * This method checks if the device is authenticated and if enough time has passed
 * since the last RSSI request. If so, it initiates a new RSSI read request to the
 * connected BLE device. The method ensures that only one RSSI request is in progress
 * at any given time.
 */
void BLEProximity::poll() {
  // DPRINTF(0, "BLEProximity::poll()");
  checkFailsafeTimeout();  // Failsafe check

  if (!deviceConnected) return;

  if (!device.isAuthenticated) return;

  const uint32_t now = millis();
  const uint32_t intervalMs = 900;  // 1 RSSI-measurement per 900ms

  // Wait until previous request is done
  if (rssiRequestInProgress) return;

  // Only start a new request if the interval has passed
  if (now - lastRssiRequestMs < intervalMs) return;

  // request RSSI if authenticated
  BLEAddress addr(device.data.mac.c_str());
  esp_err_t err = esp_ble_gap_read_rssi((uint8_t*)addr.getNative());

  if (err == ESP_OK) {
    rssiRequestInProgress = true;
    lastRssiRequestMs = now;
  } else {
    DPRINTF(2, "esp_ble_gap_read_rssi failed (err=%d) for %s", err, addr.toString().c_str());
    lastRssiRequestMs = now;  // little cooldown so we don't keep spamming
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
 * This method sends the current switch state to the `switchChar` characteristic
 * It uses the `notifyChar` helper function to perform the notifications.
 *
 * @param state The current state of the switch, represented as a string ("OPEN" or "CLOSED").
 */
void BLEProximity::notifySwitch(const char* state) {
  // notifyChar(cmdChar, state);
  notifyChar(switchChar, state);
}

/**
 * @brief Enqueues a command for processing by the command worker task.
 *
 * This method creates a `ProximityCommand` structure with the specified command
 * string and source, and attempts to enqueue it into the `commandQueue`. If the
 * queue is full, the command is dropped and a warning message is logged.
 *
 * @param cmd The command string to enqueue.
 * @param src The source of the command (Internal or External).
 */
void BLEProximity::enqueueCommand(const std::string& cmd, CommandSource src) {
  if (!commandQueue) {
    DPRINTF(3, "Command queue not initialized");
    return;
  }

  ProximityCommand pc{};
  strncpy(pc.value, cmd.c_str(), CMD_MAX_LEN - 1);
  pc.value[CMD_MAX_LEN - 1] = '\0';  // Ensure null-termination
  pc.source = src;

  if (xQueueSendToBack(commandQueue, &pc, 0) != pdPASS) {
    DPRINTF(2, "Command queue full, dropping command: %s", cmd.c_str());
  }
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
    notifyChar(cmdChar, "Invalid command");
    return;
  }

  if (value == "status") {
    bool state = digitalRead(device.getSwitchPin()) == HIGH;
    // notifyChar(cmdChar, stateToStr(state));
    notifyChar(switchChar, stateToStr(state));
    DPRINTF(1, "Switch: %s", stateToStr(switchState));
    return;
  }

  // Helper to enqueue a switch notification for switchChar
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

  // Non-momentary: direct notify via task
  if (value == "open" || value == "close" || value == "toggle") {
    enqueueSwitch(target, 0);
    return;
  }

  // Momentary: two-phase notifications (cancel pending first)
  if (value == "momOpen") {
    if (sSwitchQueue) xQueueReset(sSwitchQueue);  // clear old sequences
    enqueueSwitch(true, 0);
    enqueueSwitch(false, (uint32_t)device.data.mom_switch_delay);
    DPRINTF(0, "momOpen sequence: %d ms", device.data.mom_switch_delay);
  } else if (value == "momClose") {
    if (sSwitchQueue) xQueueReset(sSwitchQueue);
    enqueueSwitch(false, 0);
    enqueueSwitch(true, (uint32_t)device.data.mom_switch_delay);
    DPRINTF(0, "momClose sequence: %d ms", device.data.mom_switch_delay);
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
  DPRINTF(0, "BLEProximity::onConnect");
  device.data.mac = BLEAddress(param->connect.remote_bda).toString();
  DPRINTF(1, "Device connected: %s", device.data.mac.c_str());
  deviceConnected = true;
  lastFailsafeActivityMs = millis();  // Activate failsafe
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
  DPRINTF(0, "BLEProximity::onDisconnect()");

  deviceConnected = false;
  lastFailsafeActivityMs = millis();

  enqueueCommand(device.data.on_disconnect_command, CommandSource::Internal);  // Enqueue the command for processing
  DPRINTF(1, "Device disconnected: %s (%s)\n\tadvertising restarted\n", device.data.name.c_str(), device.data.mac.c_str());
  delay(500);
  device.resetRuntimeState();  // Reset device to it's initial state
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
      rssiRequestInProgress = false;            // Mark that the RSSI request is complete
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
          if (device.triggerRssiUpdate) {
            DPRINTF(1, "Updating RSSI: %d -> %d dBm", device.data.rssi_threshold, param->read_rssi_cmpl.rssi);
            device.data.rssi_threshold = param->read_rssi_cmpl.rssi;  // Update RSSI in device
            device.update();                                          // Update the json file
            notifyChar(cmdChar, "RSSI updated");
          }

          uint32_t delayOffset = device.data.rssi_command == "momOpen" || device.data.rssi_command == "momClose"
                                     ? (uint32_t)device.data.mom_switch_delay
                                     : 0;
          if (device.data.rssi_command != "" && param->read_rssi_cmpl.rssi >= device.data.rssi_threshold &&
              (millis() - device.rssiExecutedTimeStamp >= (uint32_t)device.data.rssi_command_delay * 1000 + delayOffset)) {
            DPRINTF(0, "Measured RSSI %d ≥ %d.\n Executing RSSI command: %s",
                    param->read_rssi_cmpl.rssi, device.data.rssi_threshold, device.data.rssi_command.c_str());
            // cmdChar->setValue(device.data.rssi_command.c_str());       // Set the command characteristic value
            enqueueCommand(device.data.rssi_command, CommandSource::Internal);  // Enqueue the command for processing
            device.rssiExecutedTimeStamp = millis();
          }

          notifyChar(proximityChar, String(param->read_rssi_cmpl.rssi).c_str());
          // END CRITICAL SECTION
          xSemaphoreGive(device.mutex);
        } else {
          DPRINTF(2, "WARNING: Could not acquire mutex in handleGAPEvent()");
        }
      } else {
        DPRINTF(2, "RSSI read failed: Event(%d) Error(%d)", event, param->read_rssi_cmpl.status);
      }
      break;
    }
    default:
      break;
  }
}

void BLEProximity::disconnectAll() {
  DPRINTF(0, "BLEProximity::disconnectAll()");
  if (pBLEServer) {
    delay(50);
    pBLEServer->disconnect(0);  // Disconnect all connected clients
  }
}

void BLEProximity::setFailsafeTimeout(uint32_t sec) {
  DPRINTF(0, "BLEProximity::setFailsafeTimeout(%ds)", sec);
  failsafeMonitorEnabled = sec > 0 ? true : false;
  lastFailsafeActivityMs = millis();
  failsafeTimeoutMs = sec * 1000UL;
}

void BLEProximity::setFailsafeCommand(std::string& cmd) {
  DPRINTF(0, "BLEProximity::setFailsafeCommand(%s)", cmd);
  if (cmd == "open" || cmd == "close") failsafeCommand = cmd;
}

// Worker task to process commands from the queue
void BLEProximity::commandWorkerTask(void* pvParameters) {
  BLEProximity* self = static_cast<BLEProximity*>(pvParameters);
  if (!self) {
    vTaskDelete(nullptr);
    return;
  }

  DPRINTF(0, "CommandWorker task started");

  ProximityCommand cmd;
  for (;;) {
    if (xQueueReceive(self->commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      // Process the command outside of BTC_TASK
      self->processCommand(cmd);
    }
  }
}

void BLEProximity::checkFailsafeTimeout() {
  if (!failsafeMonitorEnabled) {
    return;
  }

  if (lastFailsafeActivityMs == 0) {
    return;
  }

  uint32_t now = millis();
  if (now - lastFailsafeActivityMs < failsafeTimeoutMs) {
    // Timeout not yet reached
    return;
  }

  // Timeout reached → close/open switch
  DPRINTF(1, "Safety monitor timeout reached. Disconnect all Switch → %s", failsafeCommand.c_str());
  setSwitchState(failsafeCommand);

  if (deviceConnected) {
    disconnectAll();
    lastFailsafeActivityMs = millis();  // Fire one last time after all devices are disconnected
  } else
    lastFailsafeActivityMs = 0;  // Disable failsafe until a device connects
}

ProximitySecurity::ProximitySecurity(ProximityDevice& devData) : device(devData) {}

uint32_t ProximitySecurity::onPassKeyRequest() {
  uint32_t currentPasskey = random(100000, 999999);
  DPRINTF(1, "Generated passkey: %06u", currentPasskey);

  return currentPasskey;
}

/** Passkey notify handler */
void ProximitySecurity::onPassKeyNotify(uint32_t passkey) {
  DPRINTF(2, "Passkey notify: %06u", passkey);
  if (s_passkeyHandler) {
    s_passkeyHandler(passkey);
  }
}

bool ProximitySecurity::onConfirmPIN(uint32_t passkey) {
  DPRINTF(0, "Confirming passkey: %06u", passkey);
  return true;
}

bool ProximitySecurity::onSecurityRequest() {
  DPRINTF(0, "Security request received");
  return true;
}

/** Set the passkey notify handler */
void ProximitySecurity::setPasskeyNotifyHandler(PasskeyNotifyHandler handler) {
  DPRINTF(0, "ProximitySecurity::setPasskeyNotifyHandler()");
  s_passkeyHandler = handler;
}

/** Set authentication result handles */
void ProximitySecurity::setAuthResultHandler(AuthResultHandler handler) {
  DPRINTF(0, "ProximitySecurity::setAuthResultHandler()");
  s_authResultHandler = handler;
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

  if (s_authResultHandler) {
    s_authResultHandler(cmpl.success);  // Notify result handler
  }

  std::string macStr = BLEAddress(cmpl.bd_addr).toString();

  if (device.data.is_blocked) {
    esp_ble_gap_disconnect(cmpl.bd_addr);  // Disconnect blocked devices
    return;
  }

  if (device.isAuthenticated) {
    DPRINTF(1, "Authentication successful");
    // printBondedDevices();
    std::string hashedKey = getHashedPeerKey(cmpl.bd_addr);
    if (hashedKey == "") {
      DPRINTF(2, "No bonded key found for device: %s", macStr.c_str());
      return;
    }

    // DPRINTF(0, "Mac %s Peer key: %s", macStr.c_str(), hashedKey.c_str()); // Do NOT log hashed key in production for security reasons
    if (!device.get(hashedKey)) {
      DPRINTF(1, "Device not found in JSON, creating new device");
      device.data.device_id = hashedKey;
      device.data.mac = macStr;
      device.data.paired = cmpl.success;

      // This will make sure the initial RSSI is added to device and saved to the device file on a proximity request event
      device.triggerRssiUpdate = true;
    } else {
      DPRINTF(0,
              "Device retrieved from JSON:\n"
              "    device.data.deviceID:\t***************\n"
              "    device.data.name:\t\t%s\n"
              "    device.data.mac:\t\t%s\n"
              "    device.data.paired:\t\t%s\n"
              "    device.data.is_blocked:\t%s\n"
              "    device.data.is_admin:\t%s\n"
              "    device.data.rssi_threshold:\t%d\n"
              "    device.data.mom_switch_delay:\t%d\n"
              "    device.data.rssi_command:\t%s\n"
              "    device.data.rssi_command_delay:\t%d\n"
              "    device.data.on_disconnect_command:\t%s",
              device.data.name.c_str(),
              device.data.mac.c_str(),
              device.data.paired ? "true" : "false",
              device.data.is_blocked ? "true" : "false",
              device.data.is_admin ? "true" : "false",
              device.data.rssi_threshold,
              device.data.mom_switch_delay,
              device.data.rssi_command.c_str(),
              device.data.rssi_command_delay,
              device.data.on_disconnect_command.c_str());
      if (device.data.mac != macStr) {
        DPRINTF(1, "MAC address updated: %s -> %s", device.data.mac.c_str(), macStr.c_str());
        device.data.mac = macStr;  // Update MAC address if it has changed
        device.update();           // Save updated device data to JSON
      }
    }
    DPRINTF(1, "\t*** Welcome %s ***", device.data.name.c_str());
  } else {
    DPRINTF(2, "Authentication failed: %s", macStr.c_str());

    // Actively disconnect the peer that failed to authenticate
    esp_err_t err = esp_ble_gap_disconnect(cmpl.bd_addr);
    if (err != ESP_OK) {
      DPRINTF(3, "Failed to disconnect device %s, err=%d", macStr.c_str(), err);
    }
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

  bool internalCall = (param == nullptr);

  std::string value = pChar->getValue();
  if (value.empty()) return;
  CommandSource src = internalCall ? CommandSource::Internal
                                   : CommandSource::External;
  bleProx->enqueueCommand(value, src);
}

/**
 * @brief Processes a proximity command received from a BLE client or internal source.
 *
 * This method handles various commands related to the proximity device, including
 * state changes, configuration updates, and status queries. It performs necessary
 * validations, updates the device state, and sends notifications back to the BLE
 * client as needed.
 *
 * @param cmd The ProximityCommand structure containing the command value and source.
 */
void BLEProximity::processCommand(const ProximityCommand& cmd) {
  const std::string value(cmd.value);
  CommandSource src = cmd.source;

  DPRINTF(0, "Processing command in worker: %s", value.c_str());

  if (cmd.source == CommandSource::External) {
    DPRINTF(0, "  Source: External (from BLE client)");
  } else {
    DPRINTF(0, "  Source: Internal (from server)");
  }

  if (!device.data.paired) {
    DPRINTF(3, "Illegal write attempt: device not paired");
    return;
  }
  if (device.data.is_blocked) {
    DPRINTF(3, "Illegal write attempt: device is blocked");
    return;
  }

  if (value.length() > 0) {
    bool isAdminMsg = false;

    // TODO: onDeleteDevice -> remove paired device from bonded list
    bool bState = (value == "open" || value == "close" || value == "toggle" ||
                   value == "momOpen" || value == "momClose" || value == "status");
    bool bSetName = String(value.c_str()).startsWith("name=");                      // device name
    bool bSetMomDelay = String(value.c_str()).startsWith("momDelay=");              // momentary switch delay
    bool bSetRssiUpdate = (value == "rssiUpdate");                                  // RSSI update command
    bool bSetRssiCmd = String(value.c_str()).startsWith("rssiCmd=");                // RSSI command
    bool bSetRssiDelay = String(value.c_str()).startsWith("rssiDelay=");            // RSSI command delay
    bool bWhoAmI = (value == "whoami");                                             // request device info
    bool bSetDisconnectCmd = String(value.c_str()).startsWith("onDisconnectCmd=");  // on disconnect command (admin)
    bool bSetFailsafeCmd = String(value.c_str()).startsWith("failsafeCmd=");        // failsafe open/close (admin)
    bool bSetFailsafeTimer = String(value.c_str()).startsWith("failsafeTimer=");    // failsafe timeout in sec (admin)
    bool bAllUsers = (value == "allUsers");                                         // request all users (admin)
    bool bFormat = (value == "format");                                             // format file system (admin)
    bool bReboot = (value == "reboot");                                             // reboot device (admin)

    bool isValidCommand = (bState || bSetName || bSetMomDelay || bSetRssiCmd ||
                           bSetRssiDelay || bSetRssiUpdate || bWhoAmI ||
                           bSetDisconnectCmd || bSetFailsafeCmd || bSetFailsafeTimer ||
                           bAllUsers || bFormat || bReboot);

    const char* invalidMsg = "Invalid command";
    if (!isValidCommand) {
      DPRINTF(3, "%s: '%s'", invalidMsg, value.c_str());
      notifyChar(cmdChar, invalidMsg);
      return;
    }

    if (bState) {
      setSwitchState(value);  // Update the switch state according to the command
      return;
    }

    // Update the device name if the command is "name="
    if (bSetName) {
      std::string newName = value.substr(5);  // Skip "name="
      if (newName.length() > 0) {
        if (device.data.name != newName) {  // Only update if the name has changed
          device.data.name = newName;
          device.update();
          notifyChar(cmdChar, ("Name set to: " + newName).c_str());
          DPRINTF(1, "Device name set to: %s", newName.c_str());
        }
        return;
      } else {
        notifyChar(cmdChar, "Invalid name");
        DPRINTF(3, "Invalid name received");
      }
    }

    if (bSetMomDelay) {
      const char* s = value.c_str() + 9;  // na "momDelay="
      char* endp = nullptr;
      long ms = strtol(s, &endp, 10);

      if (endp == s) {
        std::string msg = "Invalid momDelay payload";
        notifyChar(cmdChar, msg.c_str());
        DPRINTF(3, "%s: %s", msg.c_str(), value.c_str());
        return;
      }

      // Clamp to int16_t and practical boundaries
      if (ms < 10) ms = 10;        // minimal 10 ms to prevent '0'
      if (ms > 30000) ms = 30000;  // upper limit 30s within int16_t

      device.data.mom_switch_delay = static_cast<int16_t>(ms);
      device.update();

      char buf[48];
      snprintf(buf, sizeof(buf), "momDel=%ld", ms);
      notifyChar(cmdChar, buf);
      DPRINTF(1, "mom_switch_delay set to %ld ms", ms);
      return;
    }

    if (bSetRssiCmd) {
      std::string newCmd = value.substr(8);  // Skip "rssiCmd="
      device.data.rssi_command = newCmd;
      device.update();
      notifyChar(cmdChar, ("rssiCmd set to: " + newCmd).c_str());
      DPRINTF(1, "rssiCmd set to: %s", newCmd.c_str());
      return;
    }

    if (bSetRssiDelay) {
      const char* s = value.c_str() + 10;  // na "rssiDelay="
      char* endp = nullptr;
      long sec = strtol(s, &endp, 10);

      if (endp == s) {
        notifyChar(cmdChar, "Invalid rssiDelay value");
        DPRINTF(3, "Invalid rssiDelay payload: %s", value.c_str());
        return;
      }

      // Clamp naar int16_t en praktische grenzen
      if (sec < 0) sec = 0;        // minimaal 0s
      if (sec > 3600) sec = 3600;  // bovengrens 1 uur

      device.data.rssi_command_delay = static_cast<int16_t>(sec);
      device.update();

      char buf[48];
      snprintf(buf, sizeof(buf), "rssiDelay=%ld", sec);
      notifyChar(cmdChar, buf);
      DPRINTF(1, "rssi_command_delay set to %lds", sec);
      return;
    }

    if (bSetRssiUpdate) {
      device.triggerRssiUpdate = bSetRssiUpdate;  // RSSI update on next proximity event
      return;
    }

    // Report device info in json format
    if (bWhoAmI) {
      const std::string& info = device.getJsonDevInfo();
      notifyChar(cmdChar, info.c_str());
      // notifyChar(switchChar, info.c_str());
      DPRINTF(1, info.c_str());
      return;
    }

    if (bSetDisconnectCmd && device.data.is_admin) {
      std::string newCmd = value.substr(16);  // Skip "onDisconnectCmd="
      device.data.on_disconnect_command = newCmd;
      device.update();
      notifyChar(cmdChar, ("onDisconnectCmd set to: " + newCmd).c_str());
      DPRINTF(1, "onDisconnectCmd set to: %s", newCmd.c_str());
      return;
    } else if (bSetDisconnectCmd)
      isAdminMsg = true;

    if (bSetFailsafeCmd && device.data.is_admin) {
      std::string cmd = value.substr(strlen("failsafeCmd="));  // after "failsafeCmd="

      if (cmd != "open" && cmd != "close") {
        notifyChar(cmdChar, "Invalid failsafeCmd (use 'open' or 'close')");
        DPRINTF(3, "Invalid failsafeCmd: %s", cmd.c_str());
        return;
      }

      setFailsafeCommand(cmd);
      notifyChar(cmdChar, ("failsafeCmd set to: " + cmd).c_str());
      DPRINTF(1, "failsafeCmd set to: %s", cmd.c_str());
      return;
    } else if (bSetFailsafeCmd)
      isAdminMsg = true;

    if (bSetFailsafeTimer && device.data.is_admin) {
      const char* s = value.c_str() + strlen("failsafeTimer=");  // after "failsafeTimer="
      char* endp = nullptr;
      long sec = strtol(s, &endp, 10);

      if (endp == s || sec < 0 || sec > 86400L) {  // max 24 hours
        notifyChar(cmdChar, "Invalid failsafeTimer value");
        DPRINTF(3, "Invalid failsafeTimer payload: %s", value.c_str());
        return;
      }

      setFailsafeTimeout((uint32_t)sec);

      if (sec == 0) {
        notifyChar(cmdChar, "Failsafe disabled (timer=0)");
        DPRINTF(1, "Failsafe disabled via failsafeTimer=0");
      } else {
        std::string msg = "Failsafe timer set to " + std::to_string(sec) + " sec";
        notifyChar(cmdChar, msg.c_str());
        DPRINTF(1, msg.c_str());
      }
      return;
    } else if (bSetFailsafeTimer)
      isAdminMsg = true;

    // Publish the JSON file if requested
    if (bAllUsers && device.data.is_admin) {
      notifyChar(cmdChar, device.printJsonFile().c_str());
      return;
    } else if (bAllUsers)
      isAdminMsg = true;

    // Format the LittleFS if requested
    if (bFormat && device.data.is_admin) {
      if (!device.getFSHandle().format()) {
        notifyChar(cmdChar, "Format failed");
        DPRINTF(3, "Failed to format file system\n");
      } else {
        const char* fmtMsg = "File system formatted successfully";
        notifyChar(cmdChar, fmtMsg);
        DPRINTF(0, fmtMsg);
        ProximitySecurity::removeBondedDevices();
        delay(1000);
        esp_restart();  // Restart after format
      }
      return;
    } else if (bFormat)
      isAdminMsg = true;

    if (bReboot && device.data.is_admin) {
      notifyChar(cmdChar, "Rebooting device...");
      DPRINTF(1, "Rebooting device as per command");
      delay(1000);
      esp_restart();
      return;
    } else if (bReboot)
      isAdminMsg = true;

    if (isAdminMsg) {
      notifyChar(cmdChar, invalidMsg);  // Show "Invalid command" for non-admins
      DPRINTF(3, "Only admin can do that");
    }
  }
}