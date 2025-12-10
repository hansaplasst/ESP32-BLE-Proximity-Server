#include <Arduino.h>
// #include <BleKeyboard.h>
#include <dprintf.h>

#include "BLEProximity.h"

// #define LOG_INFO_TAG "I"  // Log tag for informational messages
const char* deviceName = DEV_NAME;

BLEProximity* proximityServer = nullptr;
ProximityDevice* proximityDevice = nullptr;

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
  proximityDevice = new ProximityDevice(deviceName);

  if (!proximityDevice->begin(GPIO_NUM_18, true)) {
    DPRINTF(3, "ProximityDevice init failed, halting");
    exit(3);
  }
  proximityServer = new BLEProximity(*proximityDevice);
  proximityServer->begin();

  // printESPInfo();  // Print ESP information for debugging

  DPRINTF(1, "%s Initialized,\n\tWaiting for client...", deviceName);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  proximityServer->poll();
}