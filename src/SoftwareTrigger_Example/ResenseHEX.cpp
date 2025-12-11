/**
 * @file ResenseHEX.cpp
 * @brief Implementation of the Resense HEX32 library
 */

#include "ResenseHEX.h"

// Define static member variables
float ResenseHEX::_forceMax = 5000.0f;
float ResenseHEX::_torqueMax = 10.0f;
float ResenseHEX::_tempMax = 150.0f;
uint16_t ResenseHEX::_softwareTriggerTimeoutMs = 300;

/**
 * @brief Constructor - initialize with a serial stream reference
 */
ResenseHEX::ResenseHEX(Stream &serial)
  : _serial(serial) {}

/**
 * @brief Complete measurement: trigger + read + validate
 */
bool ResenseHEX::meas(ResenseFrame &frame) {
  _flushInput();                        // flush UART input
  softwareTrigger();                    // trigger measurement
  frame.timestamp = millis();           // save timestamp
  if (!readFrame(frame)) return false;  // read data and return false if incorrect

  return true;
}

/**
 * @brief Read a 28-byte frame into ResenseFrame struct without modifying timestamp
 */
bool ResenseHEX::readFrame(ResenseFrame &frame) {
  uint8_t frameData[FRAME_SIZE];
  if (!readRawFrame(frameData)) return false;

  // Unpack 7 float32 values (little-endian)
  // Use memcpy to avoid alignment issues
  memcpy(&frame.fx, &frameData[0], 4);
  memcpy(&frame.fy, &frameData[4], 4);
  memcpy(&frame.fz, &frameData[8], 4);
  memcpy(&frame.mx, &frameData[12], 4);
  memcpy(&frame.my, &frameData[16], 4);
  memcpy(&frame.mz, &frameData[20], 4);
  memcpy(&frame.temperature, &frameData[24], 4);

  if (!_validateFrameData(frame)) return false;

  return true;
}

/**
 * @brief gets a uint8_t frameData of FRAME_SIZE and fills it with the available serial data
 */
bool ResenseHEX::readRawFrame(uint8_t *frameData) {
  uint8_t idx = 0;
  unsigned long start = millis();

  while (idx < FRAME_SIZE && (millis() - start) < _softwareTriggerTimeoutMs) {
    if (_serial.available()) {
      frameData[idx++] = _serial.read();
    }
  }
  if (idx != FRAME_SIZE) return false;  // timeout or partial frame

  return true;
}

/**
 * @brief checks for corrupted ResenseFrame without checking limits
 */
bool ResenseHEX::_validateFrameData(const ResenseFrame &frame) const {
  // Check for NaN or extreme values
  if (isnan(frame.fx) || isinf(frame.fx)) return false;
  if (isnan(frame.fy) || isinf(frame.fy)) return false;
  if (isnan(frame.fz) || isinf(frame.fz)) return false;
  if (isnan(frame.mx) || isinf(frame.mx)) return false;
  if (isnan(frame.my) || isinf(frame.my)) return false;
  if (isnan(frame.mz) || isinf(frame.mz)) return false;
  if (isnan(frame.temperature) || isinf(frame.temperature)) return false;

  return true;
}

/**
 * @brief Validate decoded frame values and compares them with set limits
 */
bool ResenseHEX::validateFrame(const ResenseFrame &frame) const {
  // Check for NaN or extreme values
  if (isnan(frame.fx) || isinf(frame.fx) || fabsf(frame.fx) > _forceMax) return false;
  if (isnan(frame.fy) || isinf(frame.fy) || fabsf(frame.fy) > _forceMax) return false;
  if (isnan(frame.fz) || isinf(frame.fz) || fabsf(frame.fz) > _forceMax) return false;
  if (isnan(frame.mx) || isinf(frame.mx) || fabsf(frame.mx) > _torqueMax) return false;
  if (isnan(frame.my) || isinf(frame.my) || fabsf(frame.my) > _torqueMax) return false;
  if (isnan(frame.mz) || isinf(frame.mz) || fabsf(frame.mz) > _torqueMax) return false;
  if (isnan(frame.temperature) || isinf(frame.temperature) || fabsf(frame.temperature) > _tempMax) return false;

  return true;
}

/**
   * @brief Flush all pending bytes from the UART receive buffer
   */
void ResenseHEX::_flushInput() {
  while (_serial.available() > 0) {
    _serial.read();  // discard byte
  }
}

/**
 * @brief Send pre-compiled software trigger over UART
 */
void ResenseHEX::softwareTrigger() {
  _serial.write(SOFTWARE_TRIGGER_CMD, SOFTWARE_TRIGGER_LEN);
  _serial.flush();  // Ensure it's sent immediately
}

/**
 * @brief Send pre-compiled tare command over UART
 */
void ResenseHEX::tare() {
  _serial.write(TARE_CMD, TARE_LEN);
  _serial.flush();  // Ensure it's sent immediately
}

/**
 * @brief tare the system and block until completed
 */
bool ResenseHEX::tareBlocking() {
  tare();
  uint8_t frameData[FRAME_SIZE];
  unsigned long start = millis();
  int nrReads = 0;

  // cycle through Frame reads until MIN_TARE_READS is reached or timeout
  while (nrReads < MIN_TARE_READS && (millis() - start) < MAX_TARE_TIMEOUT_MS) {
    softwareTrigger()
    if(readRawFrame()) {
      nrReads++;
    } else {
      return false;
    }
  }
  return true;
}

/**
 * @brief Send command over UART dynamically
 */
void ResenseHEX::sendCommand(const char *cmd) {
  _serial.write(cmd, strlen(cmd));
  _serial.flush();  // Ensure it's sent immediately
}

// Set force threshold
void ResenseHEX::setMaxForce(float forceMax) {
  _forceMax = forceMax;
}

// Set torque threshold
void ResenseHEX::setMaxTorque(float torqueMax) {
  _torqueMax = torqueMax;
}

// Set temperature threshold
void ResenseHEX::setMaxTemperature(float tempMax) {
  _tempMax = tempMax;
}

// Set trigger timeout
void ResenseHEX::setSoftwareTriggerTimeout(uint16_t timeoutMs) {
  _softwareTriggerTimeoutMs = timeoutMs;
}

// Get force threshold
float ResenseHEX::getMaxForce() const {
  return _forceMax;
}

// Get torque threshold
float ResenseHEX::getMaxTorque() const {
  return _torqueMax;
}

// Get temperature threshold
float ResenseHEX::getMaxTemperature() const {
  return _tempMax;
}

// Get trigger timeout
uint16_t ResenseHEX::getSoftwareTriggerTimeout() const {
  return _softwareTriggerTimeoutMs;
}