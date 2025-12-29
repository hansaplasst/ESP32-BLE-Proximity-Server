#include <Arduino.h>
#include <dprintf.h>

#include "BLEProximity.h"

const char* deviceName = "BLE Proximity Server";

fs::LittleFSFS fs2;

BLEProximity* proximityServer = nullptr;
ProximityDevice device(deviceName, fs2);

void setup() {
  Serial.begin(115200);
  delay(5000);  // Give serial console a little time to settle

  DPRINTF(1, "Setup started");

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