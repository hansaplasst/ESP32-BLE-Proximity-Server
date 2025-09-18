#include <Arduino.h>
#include <dprintf.h>

#include "BLEProximity.h"

// #define LOG_INFO_TAG "I"  // Log tag for informational messages
const char* deviceName = DEV_NAME;

BLEProximity* proximityServer = nullptr;

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ;  // Wait for serial port to connect. Needed for native USB
  }
  delay(5000);
  DPRINTF(1, "Setup started");
  Serial.setDebugOutput(true);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the RGB LED white

  proximityServer = new BLEProximity(deviceName);
  proximityServer->begin();

  // printESPInfo();  // Print ESP information for debugging
  // LittleFS.format();

  DPRINTF(1, "%s Initialized,\n\tWaiting for client...", deviceName);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static bool adminSet = true;  // TODO: Remove this after testing
  proximityServer->poll();
  if (proximityServer->device.isAuthenticated && adminSet) {
    proximityServer->device.setAdmin(adminSet);
    adminSet = false;
  }

  delay(500);
}