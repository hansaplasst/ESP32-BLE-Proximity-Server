# ESP32 BLE Proximity Server

A framework for developing Bluetooth Low Energy (BLE) proximity-based applications on the ESP32. This project demonstrates secure BLE pairing, device authorization, proximity (RSSI) reporting, and remote control features (e.g., door open/close) with persistent storage using LittleFS.

## Features

- BLE server with secure pairing (passkey authentication)
- Device authorization and persistent storage in JSON (LittleFS)
- Proximity detection via RSSI characteristic
- Remote control (door open/close/toggle) via BLE characteristic
- Device management (add, update, remove, set name)
- Admin features (filesystem format)
- Debug logging via serial

## Project Structure

```
.
├── src/           # Main source code (main.cpp, ble_server.cpp)
├── include/       # Header files (ble_server.h)
├── libdeps/       # PlatformIO library dependencies
├── README.md      # This file
├── platformio.ini # PlatformIO project configuration
└── LICENSE        # License information
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) or Arduino IDE
- ESP32 development board

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/your-username/esp32-ble-proximity-server.git
   cd esp32-ble-proximity-server
   ```

2. **Open the project** in PlatformIO or your preferred IDE.

3. **Install dependencies:**  
   PlatformIO will handle dependencies automatically.  
   If using Arduino IDE, ensure you have:

   - ESP32 board support
   - [ESP32 BLE Arduino](https://github.com/nkolban/ESP32_BLE_Arduino)
   - [ArduinoJson](https://arduinojson.org/)
   - [LittleFS_esp32](https://github.com/lorol/LITTLEFS)

4. **Configure settings:**  
   Edit `platformio.ini` or source files as needed for your board.

### Usage

1. **Build and upload** the firmware to your ESP32.
2. **Open the Serial Monitor** to view debug output.
3. **Pair with the ESP32** using a BLE-capable app (e.g., nRF Connect).
4. **Interact with BLE characteristics:**
   - Read RSSI (proximity)
   - Send commands to open/close/toggle the door
   - Set device name, request device JSON, or format filesystem (admin)

## BLE Characteristics

```
| UUID                                    | Name         | Properties      | Description                      |
|-----------------------------------------|--------------|----------------|-----------------------------------|
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e`  | RSSI         | Read/Notify    | Proximity (RSSI) value            |
| `6e400002-b5a3-f393-e0a9-e50e24dcca9e`  | Door         | Read/Write     | Door control commands             |
| `6e400003-b5a3-f393-e0a9-e50e24dcca9e`  | Door Status  | Read/Notify    | Door state and notifications      |
```

## Device Authorization

- Devices are authorized and stored in `/authorized_devices.json` on LittleFS.
- Each device entry includes MAC address, name, pairing status, admin flag, and RSSI.

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

---

For more details, see the source code and comments in [src/ble_server.cpp](src/ble_server.cpp) and [include/ble_server.h](include/ble_server.h).
