# ESP32 BLE Proximity Server

A framework for developing Bluetooth Low Energy (BLE) proximity-based applications on the ESP32. This project demonstrates secure BLE pairing, device authorization, proximity (RSSI) reporting, and remote control features (e.g., switch open/close). It stores device settings on persistent storage using LittleFS.

## Features

- BLE server with secure pairing (passkey authentication)
- Device authorization and persistent storage in JSON (LittleFS)
- Real-time Proximity notification via RSSI characteristic (`RSSI_UUID`)
- Real-time Switch notification via RSSI characteristic (`SWITCH_UUID`)
- Remote control: switch `open`/`close`/`toggle`/`momOpen`/`momClose` via BLE characteristic (`COMMAND_UUID`)
- Configurable `momOpen`/`momClose` delay (`momDelay`), Delay in ms between ON and OFF.
- Proximity command execution when a device is in range (execute `rssiCmd` when `rssi_threshold` is reached)
- Configurable `rssiCmd` delay (`rssiDelay`). Time in seconds to repeat `rssiCmd` when in device is range.
- Automatic command execution on disconnect or out-of-range (`onDisconnectCmd`)
- Device management: add, update, remove, set name, set proximity command, set proximity delay, set momOpen/momClose delay
- Admin features: filesystem format, retrieve device list as JSON
- Per-device admin rights management
- Secure storage and management of bonded devices

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

## BLE Characteristics

```
| UUID                                    | Name         | Properties     | Description                       |
|-----------------------------------------|--------------|----------------|-----------------------------------|
| `1802fdeb-5a0d-47b2-b56c-aea5e5aaf9f5`  | SERVICE_UUID | Service        | Proximity service                 |
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e`  | RSSI_UUID    | Read/Notify    | Proximity (RSSI) value            |
| `6e400002-b5a3-f393-e0a9-e50e24dcca9e`  | COMMAND_UUID | Read/Write     | Command control and notifications |
| `6e400003-b5a3-f393-e0a9-e50e24dcca9e`  | SWITCH_UUID  | Read/Notify    | Switch state                      |
```

### Usage

1. **Build and upload** the firmware to your ESP32.
2. **Open the Serial Monitor** to view debug output.
3. **Pair with the ESP32** using a BLE-capable app (e.g., nRF Connect).
4. **Interact with BLE characteristics:**
   - Read RSSI (proximity) (via `RSSI_UUID`)
   - Read Switch state and notifications (via `SWITCH_UUID`)
   - Send commands to open/close/toggle a switch (via `COMMAND_UUID`)
     - `open` — Opens the switch.
     - `close` — Closes the switch.
     - `momOpen` — Momentarily opens the switch (pulse open).
     - `momClose` — Momentarily closes the switch (pulse close).
     - `toggle` — Toggles the switch state.
     - `status` — Read switch state (via `SWITCH_UUID`).
   - Send command to configure `momOpen`/`momClose` commands (via `COMMAND_UUID`)
     - `momDelay=<ms>` — Sets delay (ms) for the `momOpen`/`momClose` commands (min 10ms, max 30s, default: 300ms)
   - Send commands to configure Proximity parameters (via `COMMAND_UUID`)
     - `rssiUpdate` — Command to update Proximity threshold. Place device within range of the server then send `rssiUpdate`
     - `rssiCmd=<open/close/momOpen/momClose/toggle>` — Sets the command to execute when RSSI above threshold
     - `rssiDelay=<sec>` — Sets delay (s) for `rssiCmd` to repeat when above threshold (min 1s, max 3600s, default: 5 seconds)
   - Send command to execute on disconnect (via `COMMAND_UUID`)
     - `onDisconnectCmd=<open/close/momOpen/momClose/toggle>` — Sets command to execute when device is disconnect/out-of-range
   - Send command to (de)activate failsafe (via `COMMAND_UUID`)
     - `failsafeCmd=<open/close>` — Sets the command to execute when failsafeTimer is reached
     - `failsafeTimer=<sec>` — Set to 0, to deactivate failsafe, else time in seconds to activate the failsafeCmd
   - Other `COMMAND_UUID` commands:
     - `setName=<name>` — Sets the device name.
     - `json` — Returns the list of authorized devices file in JSON (admin only).
     - `format` — Formats the filesystem (admin only).

## Device Authorization

- Devices are authorized and stored in `/authorized_devices.json` on LittleFS.
- Each device entry includes:
  - **name**: Default: `"unknown"`
  - **mac**: Device `MAC Address`
  - **paired**: Pairing status. Default: `false`
  - **isAdmin**: Admin flag. Default: `false`
  - **momSwitchDelay**: Switch delay for momentary switch (`momOpen`/`momClose`). Default: `300`ms
  - **rssi_threshold**: Proximity threshold. A value between `0` and `-100`. Default: `-100`
  - **rssi_command**: Command to execute when a device is in proximity. Default: `"momOpen"`
  - **momOpen**: Command to execute if device is in proximity. RSSI >= rssi_threshold.
  - **rssi_command_delay**: Delay in seconds for the rssi_command to repeat if above the threshold (min 1a, max 3600s, default: 5s)
  - **on_disconnect_command**: Command to execute if device disconnects or gets out of range. Default: `"close"`

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

---
