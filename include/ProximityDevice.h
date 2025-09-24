#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#pragma once

// https://arduinojson.org/v7/assistant
#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0
#include <ArduinoJson.h>
#include <LittleFS.h>

#include <string>

// When updating this struct, don't forget to set default values in ProximityDevice::remove
struct ProximityData {
  std::string deviceID = "";                    // Link key for the device
  std::string name = "unknown";                 // Name of the device
  std::string mac = "";                         // MAC address of the device
  bool paired = false;                          // Pairing status: true if paired, false otherwise
  bool isBlocked = false;                       // Blocked status: true if the device is blocked, false otherwise
  bool isAdmin = false;                         // Admin status: true if the device is an admin, false otherwise
  int8_t rssi_threshold = -100;                 // Proximity (RSSI) value. On bonding the rssi_threshold of the client will be set.
  int16_t momSwitchDelay = 300;                 // Delay in milliseconds for the momOpen / momClose command. Default is 300ms. Min 10ms, max 30000ms.
  std::string rssi_command = "momOpen";         // Command to execute if device is in proximity. RSSI >= rssi_threshold.
  int16_t rssi_command_delay = 5;               // Delay in seconds for the rssi_command to repeat if above the threshold. Default is 5 seconds. Min 1 second, max 3600 seconds.
  std::string on_disconnect_command = "close";  // Command to execute if device disconnects or gets out of range.
};

class ProximityDevice {
 public:
  ProximityDevice(const char* jsonFileName = "/authorized_devices.json");

  // Device Information
  ProximityData data;

  bool update();                          // Adds or updates data entry in the JSON file
  bool remove();                          // deletes data entry from JSON and clears resets the data entry to default
  bool triggerUpdateJson = false;         // Helper to trigger update in callback functions
  bool isAuthenticated = false;           // True if device is authenticated, else false
  bool get(const std::string& deviceID);  // Retrieves device from JSON if exists and updates data struct
  void setAdmin(bool value);              // set data.isAdmin to value and updates json
  std::string printJsonFile();            // Prints the contents of json file to INFO (1)

  SemaphoreHandle_t mutex;             // Mutex
  uint32_t rssiExecutedTimeStamp = 0;  // Timestamp of last rssi command sent (in millis).

 private:
  const char* fileName;
  File jsonFile;
  JsonDocument jsonDocument;
};

#endif
