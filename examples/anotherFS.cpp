#include <Arduino.h>
// #include <BleKeyboard.h>
// #include <LittleFS.h>
#include <dprintf.h>

#include "BLEProximity.h"

// #define LOG_INFO_TAG "I"  // Log tag for informational messages
const char* deviceName = DEV_NAME;

fs::LittleFSFS fs2;

BLEProximity* proximityServer = nullptr;
ProximityDevice device(deviceName, fs2);

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

  // Init device
  if (!device.begin(GPIO_NUM_18, true, "/fsDev", 10, "fsDev")) {
    DPRINTF(3, "ProximityDevice init failed, halting");
    exit(3);
  }

  // Init proximityServer
  proximityServer = new BLEProximity(device);
  proximityServer->begin();

  DPRINTF(1, "%s Initialized,\n\tWaiting for client...", deviceName);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  proximityServer->poll();
}