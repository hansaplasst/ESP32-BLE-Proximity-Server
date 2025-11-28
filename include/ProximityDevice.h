#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#pragma once

// https://arduinojson.org/v7/assistant
#ifndef ARDUINOJSON_SLOT_ID_SIZE
  #define ARDUINOJSON_SLOT_ID_SIZE 1
#endif
#ifndef ARDUINOJSON_STRING_LENGTH_SIZE
  #define ARDUINOJSON_STRING_LENGTH_SIZE 1
#endif
#ifndef ARDUINOJSON_USE_DOUBLE
  #define ARDUINOJSON_USE_DOUBLE 0
#endif
#ifndef ARDUINOJSON_USE_LONG_LONG
  #define ARDUINOJSON_USE_LONG_LONG 0
#endif

#include <ArduinoJson.h>
#include <LittleFS.h>

#include <string>

/**
 * When updating this struct, don't forget to set values in:
 *    ProximityDevice::update
 *    ProximityDevice::get
 *    ProximityDevice::remove
 */
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
  /**
   * @brief Constructs a ProximityDevice object and sets the device name, fileSystem and device settings file
   *
   * @param deviceName   Name of the Proximity Server
   * @param fileSyatem   LittleFS file system handle. Used for storing BLE device settings in a json file
   * @param jsonFileName The name of the JSON file to use for device settings.
   */
  ProximityDevice(std::string name = "BLE Proximity Server", fs::LittleFSFS& fileSystem = LittleFS, const char* jsonFileName = "/authorized_devices.json");
  ~ProximityDevice();
  ProximityDevice(const ProximityDevice&) = delete;             // Don't allow copies
  ProximityDevice& operator=(const ProximityDevice&) = delete;  // Don't allow copies

  // Device Information
  ProximityData data;

  /**
   * @brief Initializes the ProximityDevice switch, file system, base path, maximum open files and partition label.
   *
   * begin() attempts to mount the LittleFS file system. If mounting fails, it tries to format the file system.
   * It then attempts to open the JSON device settings file (@see constructor). If the file does not exist, it creates
   * a new file with an empty JSON object.
   *
   * begin() then attempts to deserialize the JSON content. If deserialization fails, it resets the file with an
   * empty JSON object and retries. Diagnostic messages are printed throughout the process.
   *
   * @param switchPin      Switch Pin to make HIGH/LOW. Default GPIO_NUM_18
   * @param formatOnFail   Format file system on fail. Default false
   * @param basePath       File system base path. Default "/littlefs"
   * @param maxOpenFiles   Maximum open files. Default 10
   * @param partitionLabel File system partition label. Default "spiffs"
   *
   * @return True on success else False
   */
  bool begin(gpio_num_t switchPin = GPIO_NUM_18, bool formatOnFail = false,
             const char* basePath = "/littlefs", uint8_t maxOpenFiles = (uint8_t)10U,
             const char* partitionLabel = "spiffs");  // Init

  std::string name;                       // Name of the Proximity Server
  void resetRuntimeState();               // Resets the device to the inital state
  bool update();                          // Adds or updates data entry in the JSON file
  bool remove();                          // deletes data entry from JSON and clears resets the data entry to default
  bool triggerUpdateJson = false;         // Helper to trigger update in callback functions
  bool isAuthenticated = false;           // True if device is authenticated, else false
  bool get(const std::string& deviceID);  // Retrieves device from JSON if exists and updates data struct
  void setAdmin(bool value);              // set data.isAdmin to value and updates json
  std::string printJsonFile();            // Prints the contents of json file to INFO (1)
  gpio_num_t getSwitchPin() const;        // Gets the GPIO pin used for the switch (default GPIO18)
  fs::LittleFSFS& getFSHandle();          // return the file system handle

  SemaphoreHandle_t mutex;             // Mutex
  uint32_t rssiExecutedTimeStamp = 0;  // Timestamp of last rssi command sent (in millis).

 private:
  bool reloadFromFile(const std::string& deviceID = "");  // Update device data from json
  const char* fileName;
  fs::LittleFSFS& fSys;
  File jsonFile;
  JsonDocument jsonDocument;
  gpio_num_t switch_pin = GPIO_NUM_18;  // GPIO pin for the switch (default to GPIO18)
};
#endif
