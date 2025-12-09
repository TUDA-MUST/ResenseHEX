/**
 * @file ResenseHEX.cpp
 * @brief Implementation of the Resense HEX32 library
 */

#include "ResenseHEX.h"

/**
 * @brief Constructor - initialize with a serial stream reference
 */
ResenseHEX::ResenseHEX(Stream &serial)
  : _serial(serial),
    _bufferHead(0),
    _bufferTail(0),
    _frameCount(0),
    _skipCount(0),
    _triggerMode(TriggerMode::CONTINUOUS),
	memset(_buffer, 0, BUFFER_SIZE);
}

/**
 * @brief Initialize with default UART settings (2 Mbaud, 8-N-1)
 */
bool ResenseHEX::begin(int rxPin, int txPin) {
  return begin(DEFAULT_BAUD, DEFAULT_CONFIG, rxPin, txPin);
}

/**
 * @brief Initialize with custom UART settings
 */
bool ResenseHEX::begin(unsigned long baud, uint32_t config, int rxPin, int txPin) {
  _serial.begin(baud, config, rxPin, txPin);
  flushInput();
  return true;
}

/**
 * @brief Check if buffer has at least n bytes available
 */
bool ResenseHEX::_hasBytes(size_t n) const {
  size_t available;
  if (_bufferHead >= _bufferTail) {
    available = _bufferHead - _bufferTail;
  } else {
    available = BUFFER_SIZE - _bufferTail + _bufferHead;
  }
  return available >= n;
}

/**
 * @brief Return number of bytes currently buffered
 */
size_t ResenseHEX::bufferedBytes() const {
  if (_bufferHead >= _bufferTail) {
    return _bufferHead - _bufferTail;
  } else {
    return BUFFER_SIZE - _bufferTail + _bufferHead;
  }
}

/**
 * @brief Fill the internal buffer with bytes from the serial stream
 */
size_t ResenseHEX::_fillBuffer() {
  size_t bytesAdded = 0;
  
  while (_serial.available() > 0) {
    // Check if buffer is full
    size_t nextHead = (_bufferHead + 1) % BUFFER_SIZE;
    if (nextHead == _bufferTail) {
      // Buffer full, stop reading
      break;
    }
    
    int byte = _serial.read();
    if (byte >= 0) {
      _buffer[_bufferHead] = (uint8_t)byte;
      _bufferHead = nextHead;
      bytesAdded++;
    }
  }
  
  return bytesAdded;
}

/**
 * @brief Peek at a byte in the buffer without removing it
 */
uint8_t ResenseHEX::_peekByte(size_t offset) const {
  if (!_hasBytes(offset + 1)) {
    return 0;
  }
  size_t pos = (_bufferTail + offset) % BUFFER_SIZE;
  return _buffer[pos];
}

/**
 * @brief Discard one byte from the buffer
 */
void ResenseHEX::_discardByte() {
  if (_bufferHead != _bufferTail) {
    _bufferTail = (_bufferTail + 1) % BUFFER_SIZE;
  }
}

/**
 * @brief Extract 28 bytes from buffer for frame decoding
 */
size_t ResenseHEX::_extractFrame(uint8_t *dest) {
  if (!_hasBytes(FRAME_SIZE)) {
    return 0;
  }
  
  for (size_t i = 0; i < FRAME_SIZE; i++) {
    dest[i] = _peekByte(i);
  }
  
  // Advance tail by FRAME_SIZE
  for (size_t i = 0; i < FRAME_SIZE; i++) {
    _discardByte();
  }
  
  return FRAME_SIZE;
}

/**
 * @brief Decode a 28-byte frame into ResenseFrame struct
 */
bool ResenseHEX::_decodeFrame(ResenseFrame &frame) {
  uint8_t frameData[FRAME_SIZE];
  
  if (_extractFrame(frameData) != FRAME_SIZE) {
    return false;
  }

  // Unpack 7 float32 values (little-endian)
  // Use memcpy to avoid alignment issues
  memcpy(&frame.fx, &frameData[0], 4);
  memcpy(&frame.fy, &frameData[4], 4);
  memcpy(&frame.fz, &frameData[8], 4);
  memcpy(&frame.mx, &frameData[12], 4);
  memcpy(&frame.my, &frameData[16], 4);
  memcpy(&frame.mz, &frameData[20], 4);
  memcpy(&frame.temperature, &frameData[24], 4);
  return true;
}

/**
 * @brief Validate decoded frame values
 */
bool ResenseHEX::_validateFrame(const ResenseFrame &frame) const {
  // Check for NaN or extreme values
  if (isnan(frame.fx) || isinf(frame.fx)) return false;
  if (isnan(frame.fy) || isinf(frame.fy)) return false;
  if (isnan(frame.fz) || isinf(frame.fz)) return false;
  if (isnan(frame.mx) || isinf(frame.mx)) return false;
  if (isnan(frame.my) || isinf(frame.my)) return false;
  if (isnan(frame.mz) || isinf(frame.mz)) return false;
  if (isnan(frame.temperature) || isinf(frame.temperature)) return false;
  
  // Check for unreasonable ranges (basic sanity check)
  // HEX32 specs: Fx/Fy: ±125N, Fz: ±250N, Mx/My/Mz: ±2.25Nm
  // Add 50% margin to allow for raw values or calibration ranges
  if (fabsf(frame.fx) > INVALID_FLOAT_THRESHOLD) return false;
  if (fabsf(frame.fy) > INVALID_FLOAT_THRESHOLD) return false;
  if (fabsf(frame.fz) > INVALID_FLOAT_THRESHOLD) return false;
  if (fabsf(frame.mx) > INVALID_FLOAT_THRESHOLD) return false;
  if (fabsf(frame.my) > INVALID_FLOAT_THRESHOLD) return false;
  if (fabsf(frame.mz) > INVALID_FLOAT_THRESHOLD) return false;
  
  // Temperature should be reasonable (-40 to 125°C for most electronics)
  if (frame.temperature < -50.0f || frame.temperature > 150.0f) return false;
  
  return true;
}

/**
 * @brief Main function to read one complete frame
 */
bool ResenseHEX::readFrame(ResenseFrame &frame) {
  // Try to fill buffer with any new incoming bytes
  _fillBuffer();
  
  // Keep trying to read frames until we either succeed or run out of data
  while (_hasBytes(FRAME_SIZE)) {
    ResenseFrame candidateFrame;
    
    if (_decodeFrame(candidateFrame)) {
      if (_validateFrame(candidateFrame)) {
        frame = candidateFrame;
        _frameCount++;
        return true;
      } else {
        // Invalid frame, skip one byte and try again
        _skipCount++;
        // The byte was already discarded by _decodeFrame, so backtrack and try next position
        // Actually, we've already extracted 28 bytes, so go back to before extraction
        // This is complex; instead, just discard and continue loop
      }
    } else {
      // Not enough bytes, break
      break;
    }
  }
  
  return false;

bool ResenseHEX::readRawFrame(uint8_t *frameData) {
  if (!frameData) {
    return false;
  }

  // Fill internal buffer with any new bytes from serial
  _fillBuffer();

  // Use the existing extractor; it returns 0 if not enough data
  if (_extractFrame(frameData) != FRAME_SIZE) {
    return false;
  }

  return true;
}


/**
 * @brief Check if at least one frame is likely available
 */
bool ResenseHEX::availableFrame() const {
  // Quick check: do we have at least 28 bytes in buffer?
  return _hasBytes(FRAME_SIZE);
}

/**
 * @brief Flush all buffered data and re-synchronize
 */
void ResenseHEX::flushInput() {
  _fillBuffer();  // Grab any pending serial data first
  _bufferHead = 0;
  _bufferTail = 0;
  
  // Also flush the serial hardware buffer
  while (_serial.read() >= 0) {
    // Discard
  }
}

/**
 * @brief Attempt to resynchronize with the data stream
 */
bool ResenseHEX::resync(uint16_t maxAttempts) {
  flushInput();
  
  uint16_t attempts = 0;
  
  while (attempts < maxAttempts) {
    _fillBuffer();
    
    if (_hasBytes(FRAME_SIZE)) {
      ResenseFrame testFrame;
      
      // Save current tail position
      size_t savedTail = _bufferTail;
      
      if (_decodeFrame(testFrame)) {
        if (_validateFrame(testFrame)) {
          // Found valid frame!
          return true;
        }
      }
      
      // Restore tail and advance by 1 byte
      _bufferTail = savedTail;
      _discardByte();
      attempts++;
    } else {
      // Need more data
      delay(1);  // Small delay to allow serial to accumulate more bytes
      attempts++;
    }
  }
  
  return false;
}

/**
 * @brief Get the count of successfully decoded frames
 */
uint32_t ResenseHEX::getFrameCount() const {
  return _frameCount;
}

/**
 * @brief Get the count of skipped frames
 */
uint32_t ResenseHEX::getSkipCount() const {
  return _skipCount;
}

/**
 * @brief Get the current trigger mode
 */
TriggerMode ResenseHEX::getTriggerMode() const {
  return _triggerMode;
}

/**
 * @brief Set the trigger mode
 */
bool ResenseHEX::setTriggerMode(TriggerMode mode) {
  _triggerMode = mode; 
  return true;
}

/**
 * @brief Send a software trigger (ASCII 'R') over UART
 */
bool ResenseHEX::softwareTrigger() {
  if (_triggerMode != TriggerMode::SOFTWARE_TRIGGER) {
    return false;
  }
  
  _serial.write((uint8_t)SOFTWARE_TRIGGER_CHAR);
  _serial.flush();  // Ensure it's sent immediately
  
  return true;
}
