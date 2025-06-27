/**
 * @file ble_server.h
 * @brief BLE Server utilities for ESP32.
 *
 * Provides functions to initialize a BLE server, manage authorized devices,
 * and interact with BLE clients (e.g., proximity requests and switch state).
 *
 * Usage:
 *   - Include this header: #include "ble_server.h"
 *   - Initialize BLE server: initBLEServer("DeviceName");
 *   - Request proximity: requestProximity(BLEAddress("AA:BB:CC:DD:EE:FF"));
 *   - Check switch state: getSwithState(BLEAddress("AA:BB:CC:DD:EE:FF"));
 *
 * Paired devices are stored in a JSON file and managed automatically.
 *
 * Dependencies:
 *   - ESP32 BLE library
 *   - <map>, <string>
 */
#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <BLEAddress.h>

#include <string>

#define SERVICE_UUID "1802fdeb-5a0d-47b2-b56c-aea5e5aaf9f5"  // Service UUID
#define RSSI_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"     // Proximity characteristic
#define COMMAND_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // Command control characteristic
#define SWITCH_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // Switch characteristic

// Map of connected devices with their pairing status and initial Proximity distance
struct deviceInfo {
  std::string keyHash = "";              // Link key for the device
  std::string name = "unknown";          // Name of the device
  std::string mac = "";                  // MAC address of the device
  bool paired = false;                   // Pairing status: true if paired, false otherwise
  bool isAdmin = false;                  // Admin status: true if the device is an admin, false otherwise
  int8_t rssi_threshold = -100;          // Proximity (RSSI) value. Value will be set by the client on first connection.
  std::string rssi_command = "momOpen";  // Command to execute if RSSI >= rssi_threshold. Default is "momOpen" command.
  int16_t momSwitchDelay = 300;          // Delay in milliseconds for the momOpen / momClose command. Default is 300ms.
};

void initBLEServer(const char* deviceName = "BLE Proximity Server");  // Initialize the BLE server with a given device name.
void requestProximity(BLEAddress peerAddress);                        // Publishes the (Proximity) RSSI value to the RSSI characteristic.
bool getSwitchState(BLEAddress peerAddress);                          // Check the state of the switch for a given BLE address.
deviceInfo* getDeviceInfo();                                          // Returns a pointer to the deviceInfo struct for the currently connected device.
deviceInfo* getAuthorizedDeviceFromJson(const std::string& keyHash);  // Retrieve the authorized device information from the JSON file using the key hash.
void removeFromJson(deviceInfo* device);                              // Remove a device from the authorized devices JSON file.
void addToJson(deviceInfo* device);                                   // Add or update a device in the authorized devices JSON file.
void printESPInfo();                                                  // Print ESP32 information to the console for debugging purposes.

#endif  // BLE_SERVER_H
