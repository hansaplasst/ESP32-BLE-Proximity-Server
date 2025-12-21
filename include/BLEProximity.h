#ifndef BLE_PROXIMITY_H
#define BLE_PROXIMITY_H

#pragma once
#include <Adafruit_NeoPixel.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "ProximityDevice.h"

#define SERVICE_UUID "1802fdeb-5a0d-47b2-b56c-aea5e5aaf9f5"  // Service UUID
#define RSSI_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"     // Proximity characteristic (R)
#define SWITCH_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // Switch characteristic (R)
#define COMMAND_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // Command control characteristic (R/W)

static bool notifyChar(BLECharacteristic* pChar, const char* value);

static BLECharacteristic* cmdChar = nullptr;        // Characteristic (R/W) to receive commands and get results
static BLECharacteristic* switchChar = nullptr;     // Characteristic (R) to read switch status
static BLECharacteristic* proximityChar = nullptr;  // Characteristic (R) to read proximity status

enum class CommandSource {
  Internal,
  External
};

constexpr size_t CMD_MAX_LEN = 64;
struct ProximityCommand {
  char value[CMD_MAX_LEN];  // the raw command string ("open", "format", "name=...")
  CommandSource source;     // Internal (onDisconnect/RSSI) or External (client write)
};

class BLEProximity : public BLEServerCallbacks {
 public:
  BLEProximity(ProximityDevice& dev);  // BLEProximity instance and set ProximityDevice

  void begin();
  void poll();

  void configureStatusLed(uint8_t ledPin, bool hasRgbLed, uint8_t brightness);  // Configure built-in LED or NeoPixel for visual feedback
  void updateSwitchLed(bool isOpen);                                            // Update the built-in LED or NeoPixel to reflect switch state
  void setProximityThreshold(int8_t rssi);                                      // Initialize Proximity threshold (optional)
  void setSwitchState(const std::string& value);                                // "OPEN" or "CLOSED"
  void notifySwitch(const char* state);                                         // core-1 updates the switch characteristic
  void enqueueCommand(const std::string& cmd, CommandSource src);               // Enqueue a command for processing

  // Custom command handler for commands outside the Proximity server domain.
  // Return true if handled. Optionally write a response into out (may be nullptr).
  using CustomCommandHandler =
      bool (*)(const ProximityCommand& cmd, ProximityDevice& device, char* out, size_t outLen);
  static void setCustomCommandHandler(CustomCommandHandler handler);  // Set custom command handler

  // Callbacks
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
  void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
  void handleGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
  void disconnectAll();

  // Failsafe
  void setFailsafeTimeout(uint32_t sec);
  uint32_t getFailsafeTimeout() const { return failsafeTimeoutMs / 1000UL; }
  void setFailsafeCommand(std::string& cmd);
  const std::string& getFailsafeCommand() const { return failsafeCommand; }

  ProximityDevice& device;

 private:
  void processCommand(const ProximityCommand& cmd);  // process a command from the enqueueCommand queue
  static void commandWorkerTask(void* pvParameters);
  QueueHandle_t commandQueue = nullptr;

  BLEServer* pBLEServer = nullptr;
  BLEService* pService = nullptr;
  BLEAdvertising* pAdvertising = nullptr;
  esp_ble_sec_act_t encryptionLevel;

  bool rssiRequestInProgress = false;  // true as long as there is a read_rssi in progress
  uint32_t lastRssiRequestMs = 0;      // timestamp of the last request (millis)

  bool failsafeMonitorEnabled = true;     // Enable/Disable safety monitor
  std::string failsafeCommand = "close";  // Default failsafe command
  uint32_t failsafeTimeoutMs = 1800000;   // Default 30 min (1800000ms)
  uint32_t lastFailsafeActivityMs = 0;    // Timestamp of last disconnect
  void checkFailsafeTimeout();            // To check failsafe

  uint8_t statusLedPin = 2;
  bool statusHasRgb = false;
  uint8_t statusRgbBrightness = 10;
  Adafruit_NeoPixel* rgbLed = nullptr;
  uint32_t rgbOpenColor = 0;

  static CustomCommandHandler s_customCommandHandler;
};

class ProximitySecurity : public BLESecurityCallbacks {
 public:
  ProximitySecurity(ProximityDevice& devData);

  // Callbacks
  uint32_t onPassKeyRequest() override;
  void onPassKeyNotify(uint32_t passkey) override;
  bool onConfirmPIN(uint32_t passkey) override;
  bool onSecurityRequest() override;
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override;

  // Authentication Callback Hooks
  using PasskeyNotifyHandler = void (*)(uint32_t passkey);
  using AuthResultHandler = void (*)(bool success);
  static void setPasskeyNotifyHandler(PasskeyNotifyHandler handler);
  static void setAuthResultHandler(AuthResultHandler handler);

  static std::string keyHash(const esp_bt_octet16_t& key);
  static std::string getHexString(const esp_bt_octet16_t& value);
  static std::string getHashedPeerKey(esp_bd_addr_t mac);
  static void printBondedDevices();
  static void removeBondedDevices();
  static void removeBondedDevice(esp_bd_addr_t mac);

 private:
  ProximityDevice& device;

  static PasskeyNotifyHandler s_passkeyHandler;
  static AuthResultHandler s_authResultHandler;

  static const esp_ble_bond_key_info_t* getBondedKey(esp_bd_addr_t mac);
};

class CommandCallback : public BLECharacteristicCallbacks {
 public:
  CommandCallback(BLEProximity* BLEProx) : bleProx(BLEProx) {};
  void onWrite(BLECharacteristic* pChar, esp_ble_gatts_cb_param_t* param) override;

 private:
  BLEProximity* bleProx;
};
static CommandCallback* commandCallback = nullptr;

#endif