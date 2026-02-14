# Hombli Smart Convector Heater (2000W) - Native Matter over Thread

This repository contains a custom **Native Matter over Thread** firmware for the **Hombli Smart Convector Heater (Glass Panel 2000W)**.

It replaces the original Wi-Fi firmware with a custom C++ application running on the **ESP32-C6**, allowing for fully local, low-latency control via Apple Home, Google Home, and Home Assistant without any Tuya Cloud dependency or bridges.

**Device:** [Hombli Smart Convector Heater 2000W](https://hombli.com/nl/collections/verwarming/products/hombli-slimme-convectorkachel-2000w-wit-glas)

---

## 🚀 Features

* **Connectivity:** **Matter over Thread** (Requires a Thread Border Router like HomePod Mini, Apple TV 4K, or Nest Hub v2) (can be adapted to support matter over wifi).
* **Dual Endpoints:**
    1.  **Thermostat:** Controls Power, Target Temperature (5-35°C), and monitors Room Temperature.
    2.  **Screen Switch:** A separate On/Off switch to control the device's LED display.
* **Smart "Atomic" Startup:** Implements a custom "Power-On + Delay + Force High Mode" sequence to prevent the heater from waking up in "Eco" mode (a hardware limitation of this specific heater).
* **Inverted Logic Handling:** Automatically handles the inverted logic for the screen status (where Tuya sends `0` for ON).
* **Factory Reset:** Toggle the physical power button 10 times rapidly to factory reset the Matter credentials.

---

## 🛠️ Hardware Modifications

The original Tuya Wi-Fi module was replaced with a **WT0132C6-S5** (ESP32-C6) module.

* **SoC:** Espressif ESP32-C6 (RISC-V, Zigbee/Thread/BLE/Wi-Fi 6)
* **Flash:** 4MB
* **Communication:** UART (9600 Baud)

### Wiring
The firmware uses the standard UART pins on the module:

| ESP32-C6 Pin | Connection | Function |
| :--- | :--- | :--- |
| **GPIO 16** | Heater RX | TX (Transmit) |
| **GPIO 17** | Heater TX | RX (Receive) |
| **3.3V** | 3.3V | Power |
| **GND** | GND | Ground |

> **⚠️ Warning:** Do not power the ESP32 via uart/usb while it is connected to the heater's mains-powered UART lines. The voltage potentials may differ. Flash first, then install.

---

## ⚙️ Installation & Build

### Prerequisites
* ESP-IDF v5.2.x or v5.3.x
* ESP-Matter SDK

### Configuration
This project requires specific SDK settings to support multiple endpoints and the custom partition table.

1.  **Partition Table:** A custom `partitions.csv` is used to allocate space for Matter credentials and Thread storage.
2.  **Endpoint Limit:**
    You must increase the dynamic endpoint limit in `menuconfig`:
    * `Component config` -> `ESP Matter` -> `Maximum dynamic endpoints` = **3** (or higher)
    *(Required because we use Endpoint 0 (Root), Endpoint 1 (Thermostat), and Endpoint 2 (Screen Switch)).*
3. For proper thread support without errors, set thread device type to Minimal Thread Device (FTD works but may throw errors, not tested long term)

### Build Commands

```bash
# 1. Clean the build (Essential when changing endpoint limits)
idf.py fullclean

# 2. Erase Flash (Essential to format the new partition table)
idf.py erase-flash

# 3. Flash and Monitor
idf.py flash monitor
```

## 📱 Pairing & Usage

### Apple Home
When you pair the device, it may appear as a single tile (Thermostat).
1.  Long-press the Thermostat tile.
2.  Go to **Settings (Gear Icon)** -> **Accessories**.
3.  You will see the **Thermostat** and a **Switch** (Screen).
4.  Toggle **"Show as Separate Tiles"** to control the screen independently on your dashboard.

### Google Home / Home Assistant
The device will appear as two separate entities: a Thermostat and a Switch/Outlet.

---

## 📊 Technical: Datapoint Mapping

For reference, the internal mapping handled by `tuya_driver.cpp`:

| DP ID | Function | Logic |
| :--- | :--- | :--- |
| **1** | Power | `1`=On, `0`=Off |
| **2** | Target Temp | Integer |
| **3** | Current Temp | Integer |
| **4** | Mode | `0`=High, `1`=Low, `2`=Eco |
| **101** | Screen | **Inverted:** `0`=On, `1`=Off |

---

## ⚠️ Disclaimer
This project involves modifying mains-voltage appliances.
* **Always unplug the heater** before opening it.
* The software is provided "as is", without warranty of any kind.

---
