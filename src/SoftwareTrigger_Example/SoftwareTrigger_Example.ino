/*
 * ResenseHEX32 Example - Software Trigger Mode
 */
#include "ResenseHEX.h"
const int RX_PIN = 17;  // your chosen RX pin
const int TX_PIN = 18;  // your chosen TX pin

HardwareSerial HexSerial(2);  // e.g. ESP32 UART2 // Consider also SoftwareSerial
ResenseHEX hex(HexSerial);  // pass interface to object

HexFrame frame; // struct data will be saved to, Force (fx,fy,fz) in N, torque (mx,my,mz) in mN, temperature in °C and timestamp in ms

void setup() {
  Serial.begin(115200);  // PC debugging

  HexSerial.begin( // Configure UART for HEX (user responsibility)
    ResenseHEX::DEFAULT_BAUD,    // 2 Mbaud
    ResenseHEX::DEFAULT_CONFIG,  // 8N1
    RX_PIN,
    TX_PIN);

  while (!Serial && !HexSerial) delay(10);
  Serial.println("Serial connected.");

  // Optional: Adjust validation thresholds
  hex.setMaxForce(200.0f);       // Tighter force limit
  hex.setMaxTorque(5.0f);        // Tighter torque limit
  hex.setMaxTemperature(80.0f);  // Tighter temp limit

  if(hex.tareBlocking()) {Serial.println("Taring successful."); // block until taring is completed
  } else {Serial.println("Taring failed!"); }
}

void loop() {
  if (hex.triggerAndRead(frame)) {  // trigger measurement and read HexFrame // ! only works in Triggermode, for continuous consider "readFrameAndTimestamp"
    if (hex.validateLimits(frame)) {  // check if HexFrame violates set Limits
      // successful -> print out data
      Serial.printf("Fx=%.2fN Fy=%.2fN Fz=%.2fN Mx=%.2fN My=%.2fN Mz=%.2fN Temp=%.2fN @ %lu ms\n", frame.fx, frame.fy, frame.fz, frame.mx, frame.my, frame.mz, frame.temperature, frame.timestamp);
    } else {
      Serial.print("exceeds thresholds -> tare?");
    }
  } else { Serial.println("Timeout or corrupted frame");}
  delay(100);  // Trigger every 100ms
}
