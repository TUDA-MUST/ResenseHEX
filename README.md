<img src="docs/tud_logo.gif" align="right" width="140"/>

# Resense-HEX Arduino Library
[![Badge_must](https://img.shields.io/badge/Made_by-MUST-blue)](https://www.etit.tu-darmstadt.de/must/home_must/index.en.jsp)
[![Badge](https://img.shields.io/badge/Built%20w%2F-Arduino-blue)](https://www.arduino.cc/)

A clean, MCU-agnostic Arduino library for communicating with the **Resense HEX32** 6-axis Force/Torque Sensor over UART.

## Features

- **MCU Agnostic**: Works with ESP32, ESP32-S3, Arduino Uno/Mega, and any board with `HardwareSerial` / `SofwareSerial` or `Stream` support
- **High Speed**: Full 2 Mbaud UART communication
- **Robust Parsing**: Circular buffer with frame synchronization and validation
- **Easy to Use**: Simple `readFrame()` API
- **Well Documented**: Full Doxygen comments, comprehensive examples
- **Low Overhead**: Minimal memory footprint, efficient circular buffering

## Hardware Requirements

### HEX Sensor Pin Connections

The HEX evaluation electronics box has a 5-pin UART connector which follows the Micro-USB standard:

| Pin | Micro-USB   | Cable Color  | HEX / UART             |
|-----|-------------|--------------|------------------------|
| 1   | VCC (+5V)   | Red          | VCC (+5V)              |
| 2   | D-          | White        | TX (+3.3V)             |
| 3   | D+          | Green        | RX (+3.3V)             |
| 4   | ID          | N/A / Yellow | Trigger (optional)     |
| 5   | GND         | Black        | Ground                 |

** before you start don't forget to move the slide switch from USB to UART ** 

### Level Shifting

If your MCU is **5V-based** (Arduino Uno, Mega), use a level shifter:
- **Option 1**: TTL logic level converter
- **Option 2**: Voltage divider on HEX TX line (RX to MCU)

The 3.3V input (RX) can sometimes tolerate 5V directly due to input clamping, but level shifting is recommended to prevent damage.

### UART Configuration

```
Baud Rate:  2,000,000 bit/s
Data Bits:  8
Parity:     None
Stop Bits:  1
Flow:       None
```

## Installation

### Method 1: Arduino IDE Library Manager (when published)
```
Sketch → Include Library → Manage Libraries
Search: "ResenseHEX"
Install
```

### Method 2: Manual Installation
1. Create folder: `~/Arduino/libraries/ResenseHEX32/`
2. Copy these files into that folder:
   - `ResenseHEX.h`
   - `ResenseHEX.cpp`
3. Restart Arduino IDE

## Quick Start

### Minimal Example

```cpp
#include <HardwareSerial.h>
#include "ResenseHEX32.h"

HardwareSerial SensorSerial(1);
ResenseHEX hex(SensorSerial);

void setup() {
  Serial.begin(115200);
  hex.begin(2000000, SERIAL_8N1, 16, 17);  // RX=16, TX=17
}

void loop() {
  ResenseFrame frame;
  if (hex32.readFrame(frame)) {
    Serial.println(frame.fx);  // Force X
    Serial.println(frame.fy);  // Force Y
    Serial.println(frame.fz);  // Force Z
    Serial.println(frame.mx);  // Torque X
    Serial.println(frame.my);  // Torque Y
    Serial.println(frame.mz);  // Torque Z
    Serial.println(frame.temperature);
  }
}
```

### Details
There are essentially two different modes the Resense sensors can operate under:
1. Continuous-Mode (meaning a set frequency)
2. Trigger-Mode
	- SoftwareTrigger (triggers a measurement by sending a command over UART)
	- HardwareTrigger (triggers a measurement with a falling edge on Pin 4 / Trigger)



<img src="docs/eval_box.png" align="left" width="500"/>

| ID | Function           | Description |
|----|--------------------|-------------|
| 1  | Tara               | Pressing the tara button for more than 20 seconds switches the box to bootloader mode and makes it appear as USB storage on a normal computer; new firmware can then be flashed by dragging and dropping the supplied `.uf2` file, and after a power cycle the new firmware runs on the box. |
| 2  | Micro USB Port     | ** USB **: The native micro USB interface from the internal RP2040 microcontroller is used to connect the electronics board with the desktop PC (12 Mbit), and the electronics are supplied by 5 V from the USB interface. \ ** UART **: The UART interface is used to communicate with an external microcontroller at 2 Mbit, with the respective pin assignment described in the manual. |
| 3  | Interface Selection| A switch selects USB or UART as the active interface: \ left switch position corresponds to USB \ right switch position corresponds to UART |
| 4  | Filter             | An LED indicates filter (moving average of 10 unweighted samples) status: \  LED on: filter is enabled \ LED off: filter is disabled. |
| 5  | Switch Mode        | (* * *) 100 Hz sampling rate \   2 to 500 Hz sampling rate, and 3 to 1 kHz sampling rate, while trigger mode is also supported and the supplied SIM card tool can be used to press the buttons on the back control panel. |
| 6  | Matrix             | An LED indicates whether the calibration matrix is enabled, where LED on means output data are forces and torques in the three Cartesian axes in SI units (newtons and millinewton-meters), and LED off means raw signals from each axis that do not directly correspond to forces and torques in the Cartesian axes. |

