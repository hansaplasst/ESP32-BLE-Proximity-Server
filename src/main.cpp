#include <Arduino.h>
#include <dprintf.h>

#include "BLEProximity.h"

// #define LOG_INFO_TAG "I"  // Log tag for informational messages
const char* deviceName = DEV_NAME;

BLEProximity* proximityServer;

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
  static bool adminSet = true;
  proximityServer->poll();
  if (proximityServer->device.isAuthenticated && adminSet) {
    proximityServer->device.setAdmin(adminSet);
    adminSet = false;
  }

  // Wil je deze bestanden controleren op fouten, correcte structuur en conventies?

  // Ik vraag me af of ik de afhandeling van device juist heb aangepakt?
  // Ik heb ook wat moeite met de implementatie van addToJson. Heb ik dit juist aangepakt?
  // Verder vraag ik me af of getAuthorizedDeviceFromJson ook anders moet?

  // Ik wil dat handleGAPEvent - ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT slechts een keer mag
  // worden uitgevoerd. Dit moet volgens mij met een mutex, maar ik weet niet meer precies hoe.
  delay(5000);
}