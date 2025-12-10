#include "ProximityDevice.h"

#include <dprintf.h>

/**
 * @brief Constructs a ProximityDevice object and sets the device name, fileSystem and device settings file
 *
 * @param deviceName   Name of the Proximity Server
 * @param fileSystem   LittleFS file system handle. Used for storing BLE device settings in a json file
 * @param jsonFileName The name of the JSON file to use for device settings.
 */
ProximityDevice::ProximityDevice(std::string deviceName,
                                 fs::LittleFSFS& fileSystem,
                                 const char* jsonFileName) : name(deviceName), fSys(fileSystem), devicesFile(jsonFileName) {
  DPRINTF(0, "ProximityDevice::ProximityDevice()");
}

ProximityDevice::~ProximityDevice() {
  DPRINTF(0, "ProximityDevice::~ProximityDevice()");
  if (jsonFile) {
    jsonFile.close();
  }

  if (mutex) {
    vSemaphoreDelete(mutex);
  }
}

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
bool ProximityDevice::begin(gpio_num_t switchPin, bool formatOnFail,
                            const char* basePath, uint8_t maxOpenFiles,
                            const char* partitionLabel) {
  DPRINTF(0, "ProximityDevice::begin");

  switch_pin = switchPin;
  pinMode(switch_pin, OUTPUT);

  mutex = xSemaphoreCreateMutex();
  if (!mutex) {
    DPRINTF(3, "Failed to create mutex!");
    return false;
  }

  // Checking file system
  if (!fSys.begin(formatOnFail, basePath, maxOpenFiles, partitionLabel)) {
    DPRINTF(3, "Failed to mount %s, trying to format...\n", basePath);
    if (!fSys.format()) {
      DPRINTF(3, "Failed to format LittleFS\n");
      return false;
    }
    DPRINTF(1, "%s formatted successfully, remounting...", basePath);
    if (!fSys.begin(formatOnFail, basePath, maxOpenFiles, partitionLabel)) {
      DPRINTF(3, "Remount after format failed\n");
      return false;
    }
  } else {
    DPRINTF(0, "%s mounted successfully", basePath);
  }

  if (!devicesFile) {
    DPRINTF(3, "File name needed. Got %s", devicesFile);
    return false;
  }

  jsonFile = fSys.open(devicesFile, "r");
  if (!jsonFile) {
    DPRINTF(3, "Failed to read file: %s.\n Trying to create.", devicesFile);
    jsonFile = fSys.open(devicesFile, "w");
    if (!jsonFile) {
      DPRINTF(3, " Fail..");
      return false;
    }
    jsonFile.print("{}");
    jsonFile.close();
    DPRINTF(1, " Success.");
    jsonFile = fSys.open(devicesFile, "r");
  }
  jsonFile.close();

  jsonFile = fSys.open(devicesFile, "r+");
  if (!jsonFile) {
    DPRINTF(3, "This should not happen!!!");
    return false;
  }

  DeserializationError err = deserializeJson(jsonDocument, jsonFile);  // Try to read JSON
  if (err) {
    DPRINTF(3, "JSON read error: %s. Start with empty doc.", err.c_str());
    jsonFile.print("{}");
  }

  jsonFile.seek(0);                                        // reset to start of file
  if (err) err = deserializeJson(jsonDocument, jsonFile);  // retry to read JSON again
  if (err) {
    DPRINTF(3, "JSON read error: %s. File is not JSON.", err.c_str());
    return false;
  }
  jsonFile.close();

  return true;
}

void ProximityDevice::resetRuntimeState() {
  DPRINTF(0, "ProximityDevice::resetRuntimeState");

  // Reset Flags
  triggerUpdateJson = false;
  isAuthenticated = false;
  rssiExecutedTimeStamp = 0;

  // Eventueel ook de data resetten (zoals je bij remove() doet, maar zonder JSON te wijzigen)
  data.deviceID.clear();
  data.name = "unknown";
  data.mac.clear();
  data.paired = false;
  data.is_blocked = false;
  data.is_admin = false;
  data.rssi_threshold = -100;
  data.mom_switch_delay = 300;
  data.rssi_command = "momOpen";
  data.rssi_command_delay = 5;
  data.on_disconnect_command = "close";
}

/**
 * @brief Adds or updates the data entry in the JSON file.
 *
 * The function performs the following steps:
 * 1. Attempts to read the JSON file from LittleFS.
 * 2. If the file does not exist or is invalid, it starts with an empty JSON document.
 * 3. Checks if the device (by deviceID) already exists in the JSON.
 *    - If it exists, updates the device information.
 *    - If not, adds a new device entry.
 * 4. Writes the updated JSON document back to the file.
 *
 * @note The function assumes that LittleFS is mounted and available.
 * @note The file path is specified by devicesFile.
 */
bool ProximityDevice::update() {
  triggerUpdateJson = false;
  DPRINTF(0, "ProximityDevice::update(%s)", data.deviceID.c_str());

  if (!jsonDocument.is<JsonObject>()) {
    DPRINTF(3, "JSON document is not an object, creating new object");
    jsonDocument.clear();
    jsonDocument.to<JsonObject>();
  }

  JsonObject obj = jsonDocument[data.deviceID.c_str()];

  if (obj) {
    DPRINTF(1, "Updating existing device: %s", data.name.c_str());
  } else {
    DPRINTF(1, "Adding new device: %s", data.name.c_str());
    obj = jsonDocument[data.deviceID.c_str()].to<JsonObject>();
  }

  obj["name"] = data.name;
  obj["mac"] = data.mac;
  obj["paired"] = data.paired;
  obj["is_blocked"] = data.is_blocked;
  obj["is_admin"] = data.is_admin;
  obj["rssi_threshold"] = data.rssi_threshold;
  obj["mom_switch_delay"] = data.mom_switch_delay;
  obj["rssi_command"] = data.rssi_command;
  obj["rssi_command_delay"] = data.rssi_command_delay;
  obj["on_disconnect_command"] = data.on_disconnect_command;

  // Write to file
  jsonFile = fSys.open(devicesFile, "w");
  if (!jsonFile) {
    DPRINTF(3, "Failed to open file %s for writing", devicesFile);
    return false;  // Return if file cannot be opened
  }

  serializeJsonPretty(jsonDocument, jsonFile);  // TODO: Don't do pretty
  jsonFile.close();

  DPRINTF(1, "JSON saved to %s", devicesFile);
  return true;
}

bool ProximityDevice::remove() {
  DPRINTF(0, "ProximityDevice::remove(%s)", data.deviceID.c_str());

  if (!jsonDocument.is<JsonObject>()) {
    DPRINTF(3, "JSON document is not an object");
    return false;
  }

  if (!jsonDocument[data.deviceID.c_str()].is<JsonObject>()) {
    DPRINTF(0, "Device %s not found for deletion", data.deviceID.c_str());
    return false;
  }

  jsonDocument.remove(data.deviceID.c_str());

  // Reset data to default values
  data.deviceID.clear();
  data.name = "unknown";
  data.mac.clear();
  data.paired = false;
  data.is_blocked = false;
  data.is_admin = false;
  data.rssi_threshold = -100;
  data.mom_switch_delay = 300;
  data.rssi_command = "momOpen";
  data.rssi_command_delay = 5;
  data.on_disconnect_command = "close";

  jsonFile = fSys.open(devicesFile, "w");
  if (!jsonFile) {
    DPRINTF(3, "Failed to open file %s for writing during delete", devicesFile);
    return false;
  }

  serializeJsonPretty(jsonDocument, jsonFile);  // TODO: Don't do pretty
  jsonFile.close();

  DPRINTF(1, "Device %s deleted from %s", data.deviceID.c_str(), devicesFile);
  return true;
}

bool ProximityDevice::get(const std::string& deviceID) {
  DPRINTF(0, "get(%s)", deviceID.c_str());

  if (!jsonDocument.is<JsonObject>()) {
    DPRINTF(3, "JSON document is not an object");
    return false;
  }

  JsonObject deviceObj = jsonDocument[deviceID.c_str()];
  if (!deviceObj) {
    DPRINTF(0, "Device not found");
    return false;
  }

  data.deviceID = deviceID;
  data.name = deviceObj["name"] | data.name;                                                     // Default to "unknown" if not set
  data.mac = deviceObj["mac"] | data.mac;                                                        // Default to empty string if not set
  data.paired = deviceObj["paired"] | data.paired;                                               // Default to false if not set
  data.is_blocked = deviceObj["is_blocked"] | data.is_blocked;                                      // Default to false if not set
  data.is_admin = deviceObj["is_admin"] | data.is_admin;                                            // Default to false if not set
  data.rssi_threshold = deviceObj["rssi_threshold"] | data.rssi_threshold;                       // Default to -100 if not set
  data.mom_switch_delay = deviceObj["mom_switch_delay"] | data.mom_switch_delay;                 // Default to 300ms if not set
  data.rssi_command = deviceObj["rssi_command"] | data.rssi_command;                             // Default to "momOpen" if not set
  data.rssi_command_delay = deviceObj["rssi_command_delay"] | data.rssi_command_delay;           // Default to 5 seconds if not set
  data.on_disconnect_command = deviceObj["on_disconnect_command"] | data.on_disconnect_command;  // Default to "close" if not set

  DPRINTF(0, "Device found: %s (%s)", data.name.c_str(), data.mac.c_str());
  return true;
}

/**
 * @brief Reads the entire contents of the devicesFile file and returns it as a std::string. *
 * This function seeks to the beginning of the provided file, reads all available bytes,
 * and appends them to a std::string. After reading, it resets the file pointer to the start.
 * It also logs debug information about the operation and the file contents.
 *
 * @return std::string The contents of the file as a string.
 */
std::string ProximityDevice::printJsonFile() {
  DPRINTF(0, "ProximityDevice::printJsonFile");

  jsonFile = fSys.open(devicesFile, "r");
  if (!jsonFile) {
    DPRINTF(3, "Could not open file %s for reading", devicesFile);
    return std::string();  // Return empty string if file cannot be opened
  }

  std::string contents;
  while (jsonFile.available()) {
    contents += static_cast<char>(jsonFile.read());
  }
  jsonFile.close();
  DPRINTF(1, "Contents of %s:\n%s", devicesFile, contents.c_str());
  return contents;
}

gpio_num_t ProximityDevice::getSwitchPin() const {
  return switch_pin;
}

fs::LittleFSFS& ProximityDevice::getFSHandle() {
  return fSys;
}

bool ProximityDevice::reloadDevice(const std::string& deviceID) {
  std::string id = deviceID.empty() ? data.deviceID : deviceID;

  if (!devicesFile) {
    DPRINTF(3, "reloadDevice: devicesFile is null");
    return false;
  }

  File f = fSys.open(devicesFile, "r");
  if (!f) {
    DPRINTF(3, "reloadDevice: failed to open %s", devicesFile);
    return false;
  }

  jsonDocument.clear();
  DeserializationError err = deserializeJson(jsonDocument, f);
  f.close();

  if (err) {
    DPRINTF(3, "reloadDevice: JSON read error: %s", err.c_str());
    return false;
  }

  if (id.empty()) {
    // Alleen JSON opnieuw geladen, geen specifieke device nodig
    return true;
  }
  return get(id);
}

/**
 * @brief Sets the admin status for the proximity device.
 *
 * This method updates the internal state to reflect whether the device
 * has administrative privileges. After setting the status, it triggers
 * an update to propagate the change.
 *
 * @param value Boolean value indicating if the device should be set as admin (true) or not (false).
 */
void ProximityDevice::setAdmin(bool value) {
  DPRINTF(0, "ProximityDevice::setAdmin");
  data.is_admin = value;
  update();
}