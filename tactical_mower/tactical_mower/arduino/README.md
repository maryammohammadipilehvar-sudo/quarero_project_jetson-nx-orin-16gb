# Arduino / ESP32 Firmware (`arduino/`)

ESP32 firmware for:

- **PS5 controller → ESP32 (Bluetooth) → UART → Jetson**
- **Relay / GPIO outputs** (light, charging, siren, GPS, etc.)

Source: `arduino/PS5_ESP32/PS5_ESP32.ino`.

---

## Overview

The ESP32 connects to a PS5-compatible controller over Bluetooth using **Bluepad32**, converts joystick + trigger inputs into a small **UART telemetry/control frame**, and exposes a few **GPIO outputs** used as relay drivers (light, charging, …).

Key safety behavior:

- **Deadman switch (L1)**: drive/joystick frames are only transmitted while L1 is held.
- **Failsafe on release / disconnect**: when L1 is released, or the controller disconnects, the ESP32 sends a single **all-zero frame** (with CRC).

---

## Hardware requirements

- **ESP32** (tested with the Bluepad32 Arduino core / “esp32_bluepad32” board package)
- **PS5 controller** (or compatible gamepad supported by Bluepad32)
- **UART connection to the Jetson** (3.3V TTL, crossed TX/RX, common GND)
- **Relay / MOSFET driver board(s)** for GPIO outputs (do not drive relays directly from ESP32 GPIO)

---

## GPIO / pin mapping (current)

Defined in `PS5_ESP32.ino`:

- **UART**
  - **ESP32 RX**: GPIO **18** (`myRX`)
  - **ESP32 TX**: GPIO **15** (`myTX`)
  - **Baud**: **19200**, 8N1 (`SERIAL_BAUD_CUSTOM`)
- **Outputs (GPIO)**
  - **Light**: GPIO **25** (`Licht`)
  - **Charging**: GPIO **33** (`Charging`)
  - **Siren**: GPIO **26** (`Sirene`)
  - **GPS status**: GPIO **5** (`GPS_Status`)
  - **GPS save**: GPIO **2** (`GPS_Save`)
  - **Lidar**: GPIO **32** (`Lidar`)
  - **Forward**: GPIO **27** (`Forward`)

All outputs are configured as OUTPUT and initialized **LOW** at boot.

---

## Controller mapping (current behavior)

The firmware reads these controller fields (via Bluepad32):

- **Steering**: `axisRX()` mapped to integer range \([-100, 100]\) with a deadzone of ±50.
- **Forward/back**: `axisY()` mapped to integer range \([-100, 100]\) with a deadzone of ±50 (and sign inverted so forward is positive in the UART frame).
- **Triggers**
  - **Right trigger**: `throttle()` mapped to \([0, 100]\)
  - **Left trigger**: `brake()` mapped to \([0, 100]\)
- **Deadman**: **L1** (`ctl->l1()`)
- **Select flag**: `miscSelect()` (sent as 0/1)
- **Light**
  - **A button** toggles a latched state `g_lichtToggleState` on the **rising edge**.
  - That state is both **sent over UART** and also **driven to GPIO 25**.
- **GPS**
  - Currently `g_gpsToggleState = ctl->x()` (i.e. **pressed-state**, not latched toggle).

Battery:

- Controller battery is sampled; if below **20%**, the controller rumbles for **1s every 30s**.

---

## UART protocol

### Framing

- **Line-based ASCII** messages.
- Each message is a comma-separated payload of integers, followed by a **CRC field**, then `\n` (Arduino `println`).

Example (payload + CRC):

- `0,10,0,0,0,1,0,3f2a\n`

### CRC

CRC is **CRC16-CCITT** with:

- **Polynomial**: `0x1021`
- **Init**: `0xFFFF`
- Calculated over the **payload only** (everything before the final comma).
- Sent as **hex** (Arduino `String(crc, HEX)`), case may vary.

### ESP32 → Jetson (telemetry / drive intent)

Payload fields (7 integers):

1. `steer` (mapRX): \([-100..100]\)
2. `speed` (mapY): \([-100..100]\)
3. `throttle` (right trigger): \([0..100]\)
4. `brake` (left trigger): \([0..100]\)
5. `select`: \(\{0,1\}\) from `miscSelect()`
6. `light`: \(\{0,1\}\) from latched `g_lichtToggleState` (A toggles)
7. `gps`: \(\{0,1\}\) from `ctl->x()` (pressed-state)

CRC field:

8. `crc_hex`

Transmission rules:

- Sent every ~**200ms** *only while* **L1 is held**.
- On **L1 release**: sends one all-zero message: `0,0,0,0,0,0,0,<crc>\n`
- On **controller disconnect**: sends the same all-zero message once.

### Jetson → ESP32 (GPIO/relay commands)

The ESP32 also listens on the same UART for newline-terminated frames with **8 integers** + CRC:

1. `steer`
2. `speed`
3. `throttle`
4. `brake`
5. `select`
6. `light` (**used**) → drives GPIO 25 and updates `g_lichtToggleState`
7. `gps` (currently parsed but not acted on)
8. `charging` (**used**) → drives GPIO 33
9. `crc_hex`

Notes:

- The implementation parses 8 integers into an array and currently uses:
  - `values[5]` for **light**
  - `values[7]` for **charging**
- The comment in the `.ino` mentions `values[5]` twice; treat the **code** as authoritative.

---

## Build & flash

### Arduino IDE (recommended)

1. Install the **Bluepad32** ESP32 board package (the sketch references the “esp32_bluepad32” core).
2. Open `arduino/PS5_ESP32/PS5_ESP32.ino`.
3. Select the board from the Bluepad32 ESP32 package (the sketch comments mention selecting an ESP32 dev module in that package).
4. Connect the ESP32 over USB and select the correct COM port.
5. Compile and upload.

After flashing:

- Open Serial Monitor at **115200 baud** to see connection + UART debug logs.

---

## Wiring notes (UART)

- Use **3.3V TTL UART**.
- Cross lines: **ESP32 TX (GPIO 15) → Jetson RX**, **ESP32 RX (GPIO 18) → Jetson TX**.
- Ensure a **common ground** between ESP32 and Jetson.
- If your Jetson UART pins are not 3.3V tolerant for any reason, add a level shifter (typical Jetson UART is 3.3V TTL).

---

## Troubleshooting

### Controller won’t pair / won’t reconnect

- The firmware calls `BP32.forgetBluetoothKeys()` on every boot, which forces re-pairing behavior.
- Ensure the controller is in pairing mode and the ESP32 is powered steadily.

### No UART data on the Jetson

- Confirm baud rate **19200** on both sides.
- Confirm newline-terminated frames are being received (the ESP32 uses `Serial1.println(...)`).
- Confirm TX/RX are crossed and grounds are shared.

### CRC errors on the receiver

- CRC is computed over the payload **string bytes** exactly as transmitted (comma-separated ASCII integers, no trailing spaces).
- The CRC field is **hex**, and the ESP32 parser uses `strtol(..., 16)` for incoming commands.
- Make sure your receiver normalizes the message (e.g., trims `\r`) before CRC verification.

### Light / charging toggles don’t work

- Light GPIO (25) can be changed by:
  - Local **A button toggle**, or
  - A Jetson → ESP32 command frame with `light=0/1`.
- Charging GPIO (33) is only set by Jetson → ESP32 command frames (`charging=0/1`).

---

## Development notes / known quirks

- **Deadman behavior**: while L1 is not pressed, the code currently **does not continuously stream zeros** (the “send zeros continuously” path is commented out). It only sends a **single** zero-frame on release/disconnect.
- **GPS naming vs behavior**: variables mention “toggle”, but the current implementation mirrors the **pressed-state** of the X button.
- UART receive buffer is capped at 100 chars and resets on overflow.

