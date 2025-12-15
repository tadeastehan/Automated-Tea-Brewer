<div align="center">

# 🍵 Automated Tea Brewer

### Design and Implementation of a Fully Automated ESP32-Based Tea Brewer

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-blue?logo=espressif)](https://docs.espressif.com/projects/esp-idf/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32--S3%20%7C%20ESP32--C6-orange)](https://www.espressif.com/)

`ESP32` · `Embedded Systems` · `C` · `FreeRTOS` · `LVGL` · `TMC2130` · `Stepper Motor` · `IoT`

[Repository](https://github.com/tadeastehan/Automated-Tea-Brewer) · [Documentation](#-documentation) · [Getting Started](#-project-structure)

</div>

---

## 📋 Project Annotation

This project presents the design and implementation of a **fully automated tea brewing system** based on ESP32 microcontrollers. The system consists of two ESP32 modules working in tandem:

| Module                 | Microcontroller | Function                                   |
| ---------------------- | --------------- | ------------------------------------------ |
| **Display Controller** | ESP32-S3        | Touchscreen UI using LVGL graphics library |
| **Motor Controller**   | ESP32-C6        | Stepper motor control via TMC2130 driver   |

### ✨ Key Features

- 🌡️ **IR Thermometer** - Precise water temperature monitoring
- 📏 **Laser Distance Sensor** - Teapot presence detection (VL53L0X)
- 🔥 **Induction Cooker Interface** - Automated heating control via optocoupler
- ⚙️ **Stepper Motor Control** - Precise tea bag immersion with TMC2130
- 📱 **Touchscreen Interface** - Intuitive LVGL-based user interface
- ⏱️ **Programmable Brewing** - Configurable temperature, steeping time, and movement

> The goal was to create a fully autonomous tea brewing appliance that can prepare tea with consistent quality by precisely controlling brewing temperature, steeping time, and tea bag movement.

---

## 📄 Documents

| Document                | Path |
| ----------------------- | ---- |
| 📝 Paper                | ``   |
| 🖼️ Images & Screenshots | ``   |
| 💭 Reflection           | ``   |

---

## 📁 Project Structure

```
Automated-Tea-Brewer/
├── 📂 src/                          # Source code
│   ├── 📂 tea-brewer/               # ESP32-S3 Display Controller
│   │   ├── main/
│   │   │   ├── app_main.c           # Application entry point
│   │   │   ├── uart_comm.c/h        # UART communication
│   │   │   ├── settings.c/h         # User settings management
│   │   │   └── ui/                  # LVGL user interface
│   │   ├── components/
│   │   │   ├── bsp/                 # Board Support Package
│   │   │   └── esp_lcd_touch_cst816s/
│   │   └── managed_components/      # ESP-IDF dependencies
│   │
│   └── 📂 tea-brewer-controller/    # ESP32-C6 Motor Controller
│       ├── main/
│       │   ├── main.c               # Application entry point
│       │   ├── motor/               # TMC2130 stepper control
│       │   ├── temperature_sensor/  # IR thermometer interface
│       │   ├── distance_sensor/     # VL53L0X laser sensor
│       │   ├── pot_sensor/          # Teapot detection
│       │   ├── protocol/            # ESP-to-ESP communication
│       │   ├── console/             # USB Serial debug interface
│       │   ├── scheduler.c/h        # Task scheduling
│       │   └── rtc/                 # Real-time clock
│       └── components/
│           └── vl53l0x/             # Distance sensor driver
│
├── 📂 docs/                         # Documentation
│   ├── induction-cooker/            # Induction cooker mod guide
│   └── stepper-motor-driver/        # TMC2130 wiring diagrams
│
└── 📂 reports/                      # Data & Analysis
    ├── heat-dissipation/            # Heat dissipation analysis
    │   ├── graph_maker.py
    │   ├── data/
    │   └── graphs/
    └── ir-thermometer/              # IR thermometer calibration
        ├── graph_maker.py
        ├── data/
        └── graphs/
```

---

## 📚 Documentation

Hardware setup guides are available in the `docs/` folder:

- **[Induction Cooker Modification](docs/induction-cooker/induction_cooker.md)** - Optocoupler wiring and connector setup
- **[TMC2130 Stepper Driver](docs/stepper-motor-driver/TMC2130.md)** - SPI configuration and wiring diagrams

---

<div align="center">

Made with ☕ and ❤️

</div>
