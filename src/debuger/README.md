# Automated Tea Brewer Hardware Debugger (`debuger`)

This project is a dedicated interactive debugger and diagnostic tool for:
1. **TMC2130 Stepper Motor Driver** (SPI + GPIO)
2. **D8563TS / PCF8563 Real-Time Clock** (I2C address `0x51`)
3. **VL53L0CXV0DH/1 (VL53L0X) Laser Distance Sensor** (I2C address `0x29`)
4. **MLX90614ESF-DCC Non-Contact IR Thermometer** (I2C address `0x5A`)
5. **Induction Cooker Power Control** (`PIN_INDUCTION` = `GPIO 3`)

---

## 📌 Features

### 🔥 Induction Cooker Control
- **Manual ON / OFF (`ind on` / `ind off`)**: Turns the heating plate output HIGH or LOW.
- **Toggle State (`ind` / `ind tog`)**: Quickly toggles the heating plate state between ON and OFF.
- **Momentary Pulse (`ind pulse [ms]`)**: Simulates a momentary button press (e.g. `ind pulse 200` for optocoupler button triggers).

### 🌡️ MLX90614 Non-Contact IR Thermometer Testing
- **Single Temperature Test (`temp` / `ir` / `mlx`)**: Reads ambient temp, raw object temp, and calibrated tea brewing temperature in °C and °F.
- **Live Real-Time Thermal Stream (`temp mon` / `ir mon`)**: Continuous 10 Hz thermal stream with live thermal ASCII gauge.

### 📏 VL53L0X Laser Distance Sensor Testing
- **Single Distance Measurement (`dist` / `tof` / `laser`)**: Reads accurate distance in mm, cm, and inches.
- **Live Real-Time Distance Stream (`dist mon` / `tof mon`)**: Continuous 10 Hz telemetry stream with live proximity ASCII gauge bar.

### 🕒 D8563TS / PCF8563 I2C RTC Testing
- **I2C Bus Scanner (`scan` / `i2c`)**: Scans all 127 I2C addresses and identifies connected chips (e.g. `0x51` D8563TS RTC, `0x29` VL53L0X, `0x5A` MLX90614, `0x68` DS1307).
- **RTC Diagnostic & Register Dump (`rtc` / `rtctest`)**: Probes `0x51`, dumps all 16 raw registers (`0x00`-`0x0F`), decodes status flags (VL voltage low, STOP bit, alarm/timer flags) and parsed date/time.
- **Live 1-Second Ticking Stream (`rtc mon` / `rtc stream`)**: Live stream of current seconds to verify that the 32.768 kHz crystal oscillator is running.
- **Set RTC Time (`rtc set <timestamp>` or `rtc set Y M D h m s`)**: Sets date & time and clears STOP/VL bits so counting starts.
- **Automatic Internet Time Sync (`sync_time.py`)**: Python script to sync ESP32 RTC to network/NTP time with 1 command.

### ⚙️ TMC2130 Stepper Motor Control & Tuning
- **Direct Hardware Testing & Diagnostics**:
  - SPI communication verification test (`test` / `selftest`).
  - Read driver status, StallGuard values, errors (overtemp, open load, short to ground).
  - Enable / disable motor coils directly (`enable` / `disable`).
  - Raw pulse stepping with configurable step count and pulse delays (`step <n> [delay_us]`).
- **Full Motion Control**:
  - StallGuard2-based sensorless endstop calibration (`cal` / `c`).
  - Calibration with live real-time StallGuard stream (`calmon` / `t`).
  - Fast homing (`home` / `f`).
  - Move to percentage `0% - 100%` (e.g. `50`).
  - Move to absolute step position (`pos <steps>`).
  - Immediate emergency stop (`stop` / `x`).
- **Live Telemetry Monitoring**:
  - Real-time streaming monitor (`monitor` / `m`) displaying time, position in steps, position %, StallGuard load value (0–1023), and movement state.
- **Sensorless StallGuard Tuning**:
  - Live threshold adjustment (`sgt <val>`, `+` / `-` to fine-tune).
- **Persistent Calibration**:
  - Save to NVS Flash (`save` / `w`).
  - Load from NVS Flash (`load` / `l`).
  - Erase from NVS Flash (`clear` / `z`).

---

## 🔌 Hardware Pinout

| Function | ESP32-C6 Pin | Connected Device Pin |
|---|---|---|
| **I2C SDA** | `GPIO 22` | `D8563TS SDA` |
| **I2C SCL** | `GPIO 23` | `D8563TS SCL` |
| **Motor EN** | `GPIO 2` | `TMC2130 EN` (Active LOW) |
| **Motor DIR** | `GPIO 0` | `TMC2130 DIR` |
| **Motor STEP** | `GPIO 1` | `TMC2130 STEP` |
| **Motor CS** | `GPIO 21` | `TMC2130 CS` / `SS` |
| **SPI SCLK** | `GPIO 19` | `TMC2130 SCK` |
| **SPI MOSI** | `GPIO 18` | `TMC2130 SDI` |
| **SPI MISO** | `GPIO 20` | `TMC2130 SDO` |

---

## 💻 CLI Commands Reference

Connect via USB Serial JTAG / Serial Terminal (115200 baud):

```text
================ TMC2130, RTC, TOF & IR DEBUGGER ================
[MLX90614 IR TEMPERATURE SENSOR]
  temp / ir     - Single non-contact IR temperature test
  temp mon      - Live real-time thermal telemetry stream

[VL53L0X LASER DISTANCE SENSOR]
  dist / tof    - Single laser distance measurement (mm)
  dist mon      - Live real-time distance telemetry stream

[D8563TS I2C RTC COMMANDS]
  scan / i2c    - Scan I2C bus and list all detected devices
  rtc / rtctest - Test D8563TS chip & dump all 16 registers
  rtc mon       - Live 1-second ticking stream test
  rtc set <ts | Y M D h m s> - Set custom time or Unix timestamp

[STEPPER MOTOR COMMANDS]
  c / cal       - Full calibration (finds endpoints, run WITHOUT load)
  t / calmon    - Calibration with live StallGuard monitor
  f / home      - Fast homing (returns to start endpoint)
  <0-100>       - Move to percentage (e.g. '50', '100', '0')
  pos <steps>   - Move to absolute step position (e.g. 'pos 1600')
  step <n> [us] - Raw manual step pulse test (e.g. 'step 400 300')
  x / stop      - Emergency stop & hold
  e / enable    - Enable motor driver coil power
  d / disable   - Disable motor driver (freewheel)

[DIAGNOSTICS & TUNING]
  s / status    - Print comprehensive status for motor & RTC
  m / monitor   - Start live real-time StallGuard telemetry stream
  g / sg        - Single StallGuard reading (0-1023)
  sgt <val>     - Set StallGuard threshold (-64 to +63)
  + / -         - Increase / Decrease SGT threshold
  test          - Run automated hardware self-test routine

[CALIBRATION STORAGE]
  w / save      - Save calibration parameters to NVS Flash
  l / load      - Load calibration parameters from NVS Flash
  z / clear     - Clear calibration parameters from NVS Flash
  h / help      - Print help menu
============================================================
```

---

## 🚀 Building & Flashing

From the `src/debuger` folder:

```powershell
idf.py build flash monitor -p COM6
```
