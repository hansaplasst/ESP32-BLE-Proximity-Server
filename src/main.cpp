#include <Arduino.h>
#include <dprintf.h>

#include "ble_server.h"

// #define LOG_INFO_TAG "I"  // Log tag for informational messages

#define DEVICE_NAME "BLE Proximity Server"  // Name of the BLE device

void setup() {
  Serial.begin(BAUDRATE);
  // delay(5000);  // Allow time for the serial monitor to connect
  // Serial.setDebugOutput(true);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the RGB LED white

  initBLEServer(DEVICE_NAME);

  DPRINTF(1, "%s Initialized,\n\tWaiting for client...", DEVICE_NAME);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  deviceInfo* devInfo = getDeviceInfo();
  if (devInfo) {
    if (devInfo->paired) {
      DPRINTF(1, "Requesting Proximity for %s", devInfo->mac.c_str());
      requestProximity(BLEAddress(devInfo->mac.c_str()));
    } else
      DPRINTF(1, "%s Not Paired", devInfo->mac.c_str());
  }
  delay(5000);
}