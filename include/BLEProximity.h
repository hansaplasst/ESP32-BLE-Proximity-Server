#ifndef BLE_PROXIMITY_H
#define BLE_PROXIMITY_H

#pragma once
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "ProximityDevice.h"

#define SERVICE_UUID "1802fdeb-5a0d-47b2-b56c-aea5e5aaf9f5"  // Service UUID
#define RSSI_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"     // Proximity characteristic
#define COMMAND_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // Command control characteristic
#define SWITCH_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // Switch characteristic

static bool notifyChar(BLECharacteristic* pChar, const char* value);

static BLECharacteristic* rwCharacteristic = nullptr;          // Characteristic (R/W) to receive commands and get results
static BLECharacteristic* rSwitchCharacteristic = nullptr;     // Characteristic (R) to read switch status
static BLECharacteristic* rProximityCharacteristic = nullptr;  // Characteristic (R) to read proximity status

class BLEProximity : public BLEServerCallbacks {
 public:
  BLEProximity(const char* deviceName = "BLE Proximity Server", uint8_t switchPin = GPIO_NUM_18);
  void begin();
  void poll();

  void setProximityThreshold(int8_t rssi);        // Proximity threshold instellen (optioneel)
  void setSwitchState(const std::string& value);  // "OPEN" or "CLOSED"
  void notifySwitch(const char* state);           // <-- core-1 updates the switch characteristic

  // Callbacks
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
  void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
  void handleGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
  void disconnectAll();

  ProximityDevice device;

 private:
  BLEServer* pServer = nullptr;
  BLEService* pService = nullptr;
  BLEAdvertising* pAdvertising = nullptr;
  esp_ble_sec_act_t encryptionLevel;
  std::string device_name;

  bool rssiRequestInProgress = false;  // true as long as there is a read_rssi in progress
  uint32_t lastRssiRequestMs = 0;      // timestamp of the last request (millis)
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

  using PasskeyNotifyHandler = void (*)(uint32_t passkey);
  static void setPasskeyNotifyHandler(PasskeyNotifyHandler handler);

  static std::string keyHash(const esp_bt_octet16_t& key);
  static std::string getHexString(const esp_bt_octet16_t& value);
  static std::string getHashedPeerKey(esp_bd_addr_t mac);
  static void printBondedDevices();
  static void removeBondedDevices();
  static void removeBondedDevice(esp_bd_addr_t mac);

 private:
  ProximityDevice& device;

  static PasskeyNotifyHandler s_passkeyHandler;
  static const esp_ble_bond_key_info_t* getBondedKey(esp_bd_addr_t mac);
};

class CommandCallback : public BLECharacteristicCallbacks {
 public:
  CommandCallback(BLEProximity* BLEProx);
  void onWrite(BLECharacteristic* pChar, esp_ble_gatts_cb_param_t* param) override;

 private:
  BLEProximity* bleProx;
};

static CommandCallback* commandCallback = nullptr;

#endif