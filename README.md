# ResenseHEX32 Arduino Library

A clean, MCU-agnostic Arduino library for communicating with the **Resense HEX32** 6-axis Force/Torque Sensor over UART.

## Features

- **MCU Agnostic**: Works with ESP32, ESP32-S3, Arduino Uno/Mega, SAMD, STM32, and any board with `HardwareSerial` / `SofwareSerial` or `Stream` support
- **High Speed**: Full 2 Mbaud UART communication
- **Robust Parsing**: Circular buffer with frame synchronization and validation
- **Easy to Use**: Simple `readFrame()` API
- **Well Documented**: Full Doxygen comments, comprehensive examples
- **Low Overhead**: Minimal memory footprint, efficient circular buffering

## Hardware Requirements

### HEX32 Sensor Pin Connections

The HEX32 evaluation electronics box has a 5-pin UART connector which follows the Micro-USB standard:

| Pin | Micro-USB   | Cable Color  | HEX / UART             |
|-----|-------------|--------------|------------------------|
| 1   | VCC (+5V)   | Red          | VCC (+5V)              |
| 2   | D-          | White        | UART TX (+3.3V)        |
| 3   | D+          | Green        | UART RX (+3.3V)        |
| 4   | ID          | N/A / Yellow | Trigger (optional)     |
| 5   | GND         | Black        | Ground                 |

**Note that the voltage of the supply and UART differs!**

### UART Configuration

```
Baud Rate:  2,000,000 bit/s
Data Bits:  8
Parity:     None
Stop Bits:  1
Flow:       None
```

### Level Shifting

If your MCU is **5V-based** (Arduino Uno, Mega), use a level shifter:
- **Option 1**: TXB0108, LV-TTL logic level converter
- **Option 2**: Voltage divider on HEX32 TX line (RX to MCU)

The 3.3V input (RX) can sometimes tolerate 5V directly due to input clamping, but level shifting is recommended for reliability.

## Installation

### Method 1: Arduino IDE Library Manager (when published)
```
Sketch → Include Library → Manage Libraries
Search: "ResenseHEX32"
Install
```

### Method 2: Manual Installation
1. Create folder: `~/Arduino/libraries/ResenseHEX32/`
2. Copy these files into that folder:
   - `ResenseHEX32.h`
   - `ResenseHEX32.cpp`
3. Restart Arduino IDE

### Method 3: PlatformIO
Add to `platformio.ini`:
```ini
lib_deps = 
    path/to/ResenseHEX32
```

## Quick Start

### Minimal Example (ESP32-S3)

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
