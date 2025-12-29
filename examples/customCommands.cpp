#include <Arduino.h>

#include "BLEProximity.h"

ProximityDevice proximityDevice;
BLEProximity* proximityServer;

static bool myCustomCommandHandler(const ProximityCommand& cmd, ProximityDevice& device, char* reply, size_t replyLen) {
  
  // Optional: Only accept commands coming from BLE client
  if (cmd.source != CommandSource::External) return false;

  // Compare safely 
  if (strncmp(cmd.value, "ping", CMD_MAX_LEN) == 0) {
    Serial.println("pong");
    snprintf(reply, replyLen, "%s", "pong");  // or just write nothing; framework returns "OK"
    return true;
  }

  Serial.println("Invalid custom command");
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Setup started");

  proximityDevice.begin();
  proximityServer = new BLEProximity(proximityDevice);
  proximityServer->begin();

  BLEProximity::setCustomCommandHandler(myCustomCommandHandler);  // Register Shazam BLE commands

  Serial.println("Setup completed");
}

void loop() {
  proximityServer->poll();
  delay(10);
}