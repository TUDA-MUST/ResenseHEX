/*
 * ResenseHEX32 Example - Software Trigger Mode
 */
#include "ResenseHEX.h"

const int RX_PIN = 17;  // your chosen RX pin
const int TX_PIN = 18;  // your chosen TX pin

HardwareSerial HexSerial(2);  // e.g. ESP32 UART2 // Consider also SoftwareSerial
ResenseHEX hex(HexSerial);

ResenseFrame frame; // in this struct you can save all your data, Force (fx,fy,fz), torque (mx,my,mz), temperature (in °C) and timestamp (in ms)

bool outside = false;
int i = 0;

void setup() {
  Serial.begin(115200);  // PC debugging
  while (!Serial) delay(10);

  // Configure UART for HEX32 (user responsibility)
  HexSerial.begin(
    ResenseHEX::DEFAULT_BAUD,    // 2 Mbaud
    ResenseHEX::DEFAULT_CONFIG,  // 8N1
    RX_PIN,
    TX_PIN);

  Serial.println("ResenseHEX32 ready!");

  // Optional: Adjust validation thresholds
  hex.setMaxForce(200.0f);       // Tighter force limit
  hex.setMaxTorque(5.0f);        // Tighter torque limit
  hex.setMaxTemperature(80.0f);  // Tighter temp limit
}

void loop() {
  if (hex.meas(frame)) {
    if (hex.validateFrame(frame)) {
      Serial.printf("Fx=%.2fN Fy=%.2fN Fz=%.2fN Mx=%.2fN My=%.2fN Mz=%.2fN Temp=%.2fN @ %lu ms\n", frame.fx, frame.fy, frame.fz, frame.mx, frame.my, frame.mz, frame.temperature, frame.timestamp);
      outside = false;
    } else {
      Serial.printf("Fx=%.2fN Fy=%.2fN Fz=%.2fN Mx=%.2fN My=%.2fN Mz=%.2fN Temp=%.2fN @ %lu ms\n", frame.fx, frame.fy, frame.fz, frame.mx, frame.my, frame.mz, frame.temperature, frame.timestamp);
      Serial.print("exceeds thresholds -> tare? ");
      //hex.tare();
      if(outside) {
        i++;
        Serial.print(i);
      } else {
        outside = true;
        i=0;
      }
    }
  } else {
    Serial.println("Timeout or corrupted frame");
  }

  delay(10);  // Trigger every 100ms
}
