<img src="docs/tuda_logo.png" align="right" width="150"/>

# Arduino Library for the Resense HEX sensors
[![Badge_must](https://img.shields.io/badge/Made_by-MUST-blue)](https://www.etit.tu-darmstadt.de/must/home_must/index.en.jsp)
[![Badge](https://img.shields.io/badge/Built%20w%2F-Arduino-blue)](https://www.arduino.cc/)

A clean, MCU-agnostic Arduino library for communicating with a **Resense HEX** 6-axis Force/Torque Sensor over UART. It is:
- **MCU Agnostic**: Works with ESP32, ESP32-S3, Arduino Uno/Mega, and any board with `HardwareSerial` / `SofwareSerial` or `Stream` support
- **Easy to Use**: Simple `readFrame()` and `triggerAndRead()` API
- **Well Documented**: Full Doxygen comments, comprehensive examples

Please note that this is an unofficial library and that we are not affiliated with the Resense company!

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

If your MCU is **5V-based** (Arduino Uno, Mega), use a level shifter:
- **Option 1**: TTL logic level converter
- **Option 2**: Voltage divider on HEX TX line (RX to MCU)

**Before you continue don't forget to [move the slide switch from USB to UART](#settings)!**

### UART Configuration
```
Baud Rate:  2,000,000 bit/s
Data Bits:  8
Parity:     None
Stop Bits:  1
Flow:       None
```
_This library already pre-defines the relevant UART settings, see [Minimal Example](#minimal-example) for more._

## Installation

### Method 1: Arduino IDE Library Manager
1. Sketch → Include Library → Manage Libraries
2. Search: "ResenseHEX"
3. Install

### Method 2: Manual Installation
1. Create folder: `~/Arduino/libraries/ResenseHEX32/`
2. Copy these files into that folder:
   - `ResenseHEX.h`
   - `ResenseHEX.cpp`
3. Restart Arduino IDE

## Quick Start
There are essentially three different modes the Resense sensors can operate under:
1. **Continuous-Mode** (meaning a set frequency) -> _only pre-defined sample rates_
2. Trigger-Mode
	- **Software-Trigger** (triggered by sending a command over UART) -> _only single measurements_
	- **Hardware-Trigger** (triggered by a falling edge on Pin 4 / Trigger) -> _this requires the use of a 5 pin micro usb cable/connector_

For Continuous-Mode and Hardware-Trigger-Mode ideally use the functions `readFrame` and `readFrameAndTimestamp` (the former requires you to set the timestamp yourself).
For Software-Trigger-Mode use `triggerAndRead`.

This library uses `HexFrame` structs to pass data. They consist of the following
```
struct HexFrame {
  float fx;                 ///< Force in X direction (N or raw)
  float fy;                 ///< Force in Y direction (N or raw)
  float fz;                 ///< Force in Z direction (N or raw)
  float mx;                 ///< Torque around X axis (mNm or raw)
  float my;                 ///< Torque around Y axis (mNm or raw)
  float mz;                 ///< Torque around Z axis (mNm or raw)
  float temperature;        ///< Sensor temperature (°C)
  unsigned long timestamp;}  ///< timestamp of measurement (millis)
```

### Minimal Example

_Don't forget to set the electronics to UART mode before use, see [Settings](#settings) for more._


```
#include <HardwareSerial.h>
#include <ResenseHEX.h>

HardwareSerial HexSerial(1); // UART Hardware interface 1 // consider SoftwareSerial if none left
ResenseHEX hex(HexSerial); // create ResenseHEX object by passing a Serial Interface derived from Stream
HexFrame frame; // struct data will be saved to

void setup() {
  // PC Serial Interface
  Serial.begin(115200);	
  
  // HEX Serial Interface
  HexSerial.begin(ResenseHEX::DEFAULT_BAUD, ResenseHEX::DEFAULT_CONFIG, RX_PIN, TX_PIN); 
  
  while (!Serial && !HexSerial) delay(10); Serial.println("Serial connected.");
  
  // block until taring is completed (may fail outside Software-Trigger-Mode)
  if(hex.tareBlocking()) {Serial.println("Taring successful."); 
  } else {Serial.println("Taring failed!"); }
}

void loop() {
  // trigger measurement and read HexFrame // ! only works in Triggermode, for continuous consider "readFrameAndTimestamp"
  if (hex.triggerAndRead(frame)) { 
    // check if HexFrame violates set Limits
    if (hex.validateLimits(frame)) {  // successful -> print out data
      Serial.printf("Fx=%.2fN Fy=%.2fN Fz=%.2fN Mx=%.2fmNm My=%.2fmNm Mz=%.2fmNm Temp=%.2f°C @ %lu ms\n",
      frame.fx, frame.fy, frame.fz, frame.mx, frame.my, frame.mz, frame.temperature, frame.timestamp);
    } else {
      Serial.print("Exceeds thresholds -> tare?");
    }
  } else { 
    Serial.println("Timeout or corrupted frame");
  }
  delay(100);  // Trigger every 100ms
}
```
_Look into the examples folder for comprehensive examples!_

### Settings

<img src="docs/eval_box.png" align="center" width="500"/>

| ID | Function           | Description |
|----|--------------------|-------------|
| 1  | Tara               | 🔵 taring available <br>⚪ taring in process <br> Pressing the tara button for more than 20 seconds switches the box to bootloader mode and makes it appear as USB storage on a normal computer; new firmware can then be flashed by dragging and dropping the supplied `.uf2` file, and after a power cycle the new firmware runs on the box. |
| 2  | Interface (Micro-USB)  | **USB**: The native micro USB interface from the internal RP2040 microcontroller is used to connect the electronics board with the desktop PC (12 Mbit), and the electronics are supplied by 5 V from the USB interface. <br>**UART**: The UART interface is used to communicate with an external microcontroller at 2 Mbit, with the respective pin assignment described in the manual. |
| 3  | Interface Selection| A switch selects USB or UART as the active interface: <br> ⬅️ left switch position corresponds to **USB** <br> ➡️ right switch position corresponds to **UART** |
| 4  | Filter             | Indicates status of filter (moving average of 10 unweighted samples): <br>🔵 filter is enabled <br>⚪ filter is disabled |
| 5  | Switch Mode        | 🔵⚪⚪ Trigger <br>⚪🔵⚪ 1 kHz sampling rate <br>⚪⚪🔵 500 Hz sampling rate <br>🔵🔵🔵 100 Hz sampling rate <br> |
| 6  | Matrix             | An LED indicates whether the calibration matrix is enabled: <br>🔵 output data are forces and torques in the three Cartesian axes in SI units (newtons and millinewton-meters) <br>⚪ output data are raw signals from each axis. |

