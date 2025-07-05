#include "ProximityDevice.h"

#include <dprintf.h>

/**
 * @brief Constructs a ProximityDevice object and initializes the device's configuration from a JSON file stored in LittleFS.
 *
 * This constructor attempts to mount the LittleFS file system. If mounting fails, it tries to format the file system.
 * It then attempts to open the specified JSON configuration file. If the file does not exist, it creates a new file with an empty JSON object.
 * The constructor then attempts to deserialize the JSON content into the internal document. If deserialization fails,
 * it resets the file with an empty JSON object and retries. Diagnostic messages are printed throughout the process.
 *
 * @param jsonFileName The name of the JSON file to use for device configuration.
 */
ProximityDevice::ProximityDevice(const char* jsonFileName) : fileName(jsonFileName) {
  DPRINTF(0, "ProximityDevice()");

  mutex = xSemaphoreCreateMutex();
  if (!mutex) {
    DPRINTF(3, "Failed to create mutex!");
  }

  // Checking LittleFS file system
  if (!LittleFS.begin()) {
    DPRINTF(3, "Failed to mount LittleFS\n");
    if (!LittleFS.format()) {
      DPRINTF(3, "Failed to format LittleFS\n");
      return;
    } else {
      DPRINTF(0, "LittleFS formatted successfully");
    }
  } else {
    DPRINTF(0, "LittleFS mounted successfully");
  }

  if (!fileName) {
    DPRINTF(3, "File name needed. Got %s", fileName);
    return;
  }

  jsonFile = LittleFS.open(fileName, "r");
  if (!jsonFile) {
    DPRINTF(3, "Failed to read file: %s.\n Trying to create.", fileName);
    jsonFile = LittleFS.open(fileName, "w");
    if (!jsonFile) {
      DPRINTF(3, " Fail..");
      return;
    } else {
      jsonFile.print("{}");
      jsonFile.close();
      DPRINTF(1, "Looks Successful.")
    }
  }

  jsonFile = LittleFS.open(fileName, "rw");
  if (!jsonFile) {
    DPRINTF(3, "This should not happen!!!");
    return;
  }

  DeserializationError err = deserializeJson(jsonDocument, jsonFile);  // Try to read JSON
  if (err) {
    DPRINTF(2, "JSON read error: %s. Start with empty doc.", err.c_str());
    jsonFile.print("{}");
  }

  jsonFile.seek(0);                                        // reset to start of file
  if (err) err = deserializeJson(jsonDocument, jsonFile);  // retry to read JSON again
  if (err) {
    DPRINTF(3, "JSON read error: %s. File is not JSON.", err.c_str());
  }
  jsonFile.close();
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
 * @note The file path is specified by fileName.
 */
bool ProximityDevice::update() {
  triggerUpdateJson = false;
  DPRINTF(0, "update(%s)", data.deviceID.c_str());

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
  obj["isAdmin"] = data.isAdmin;
  obj["rssi_threshold"] = data.rssi_threshold;
  obj["rssi_command"] = data.rssi_command;
  obj["momSwitchDelay"] = data.momSwitchDelay;

  // Wegschrijven naar bestand
  jsonFile = LittleFS.open(fileName, "w");
  if (!jsonFile) {
    DPRINTF(3, "Failed to open file %s for writing", fileName);
    return false;  // Return if file cannot be opened
  }

  serializeJsonPretty(jsonDocument, jsonFile);  // TODO: Don't do pretty
  jsonFile.close();

  DPRINTF(1, "JSON saved to %s", fileName);
  return true;
}

bool ProximityDevice::remove() {
  DPRINTF(0, "remove(%s)", data.deviceID.c_str());

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
  data.isAdmin = false;
  data.rssi_threshold = -100;
  data.momSwitchDelay = 300;
  data.rssi_command = "momOpen";
  data.on_disconnect_command = "close";

  jsonFile = LittleFS.open(fileName, "w");
  if (!jsonFile) {
    DPRINTF(3, "Failed to open file %s for writing during delete", fileName);
    return false;
  }

  serializeJsonPretty(jsonDocument, jsonFile);  // TODO: Don't do pretty
  jsonFile.close();

  DPRINTF(1, "Device %s deleted from %s", data.deviceID.c_str(), fileName);
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
  data.name = deviceObj["name"] | data.name;                                // Default to "unknown" if not set
  data.mac = deviceObj["mac"] | data.mac;                                   // Default to empty string if not set
  data.paired = deviceObj["paired"] | data.paired;                          // Default to false if not set
  data.isAdmin = deviceObj["isAdmin"] | data.isAdmin;                       // Default to false if not set
  data.rssi_threshold = deviceObj["rssi_threshold"] | data.rssi_threshold;  // Default to -100 if not set
  data.rssi_command = deviceObj["rssiCommand"] | data.rssi_command;         // Default to "momOpen" if not set
  data.momSwitchDelay = deviceObj["momSwitchDelay"] | data.momSwitchDelay;  // Default to 300ms if not set

  DPRINTF(0, "Device found: %s (%s)", data.name.c_str(), data.mac.c_str());
  return true;
}

/**
 * @brief Reads the entire contents of the fileName file and returns it as a std::string. *
 * This function seeks to the beginning of the provided file, reads all available bytes,
 * and appends them to a std::string. After reading, it resets the file pointer to the start.
 * It also logs debug information about the operation and the file contents.
 *
 * @return std::string The contents of the file as a string.
 */
std::string ProximityDevice::printJsonFile() {
  DPRINTF(0, "printJsonFile()");

  jsonFile = LittleFS.open(fileName, "r");
  if (!jsonFile) {
    DPRINTF(3, "Could not open file %s for reading", fileName);
    return std::string();  // Return empty string if file cannot be opened
  }

  std::string contents;
  while (jsonFile.available()) {
    contents += static_cast<char>(jsonFile.read());
  }
  jsonFile.close();
  DPRINTF(1, "Contents of %s:\n%s", fileName, contents.c_str());
  return contents;
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
  data.isAdmin = value;
  update();
}