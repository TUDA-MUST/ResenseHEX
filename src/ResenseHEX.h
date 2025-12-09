/**
 * @file ResenseHEX.h
 * @brief Arduino library for Resense HEX32 6-axis Force/Torque Sensor
 * 
 * This library provides a clean interface to read force and torque data from
 * the Resense HEX32 sensor over UART. The library is MCU-agnostic and works
 * with any Arduino-compatible board (ESP32, ESP32-S3, Arduino, STM32, etc.)
 * as long as the target provides HardwareSerial compatibility.
 * 
 * @author Your Name
 * @version 1.1.0
 * @date 2025-12-09
 * 
 * Connection (HEX32 UART pin header):
 *   Pin 1: VDD (5V) 
 *   Pin 2: TX (3.3V) -> MCU RX
 *   Pin 3: RX (3.3V) -> MCU T
 *   Pin 4: Trigger (optional)
 *   Pin 5: GND
 * 
 * Hardware Trigger (optional):
 *   MCU GPIO (any pin) -> HEX32 trigger input (3.3V or 5V pulse)
 * 
 * UART Configuration:
 *   Baud Rate: 2,000,000 bit/s (default, configurable)
 *   Data Bits: 8
 *   Parity: None
 *   Stop Bits: 1
 * 
 * Data Frame (28 bytes total):
 *   Bytes 0-3:   fx (float32)
 *   Bytes 4-7:   fy (float32)
 *   Bytes 8-11:  fz (float32)
 *   Bytes 12-15: mx (float32)
 *   Bytes 16-19: my (float32)
 *   Bytes 20-23: mz (float32)
 *   Bytes 24-27: temperature (float32)
 */

#ifndef RESENSE_HEX_H
#define RESENSE_HEX_H

#include <Arduino.h>
#include <Stream.h>     // Explicit (sometimes needed on some platforms)
#include <cstring>

/**
 * @enum TriggerMode
 * @brief Operating modes for sensor data acquisition
 */
enum class TriggerMode {
  CONTINUOUS,        ///< Continuous streaming (default, set via HEX Mode switch)
  SOFTWARE_TRIGGER,  ///< Send ASCII 'R' over UART to trigger one sample
  HARDWARE_TRIGGER   ///< Pulse GPIO pin to trigger one sample
};

/**
 * @struct ResenseFrame
 * @brief Data structure holding one complete measurement frame from HEX
 */
struct ResenseFrame {
  float fx;                    ///< Force in X direction (N or raw)
  float fy;                    ///< Force in Y direction (N or raw)
  float fz;                    ///< Force in Z direction (N or raw)
  float mx;                    ///< Torque around X axis (mNm or raw)
  float my;                    ///< Torque around Y axis (mNm or raw)
  float mz;                    ///< Torque around Z axis (mNm or raw)
  float temperature;           ///< Sensor temperature (°C)
};

/**
 * @class ResenseHEX
 * @brief Main library class for communicating with Resense HEX sensor
 * 
 * Usage (continuous mode):
 * @code
 *   HardwareSerial SensorSerial(1);
 *   ResenseHEX hex(SensorSerial);
 *   
 *   void setup() {
 *     hex.begin();  // RX=16, TX=17; uses 2 Mbaud, 8-N-1
 *   }
 *   
 *   void loop() {
 *     ResenseFrame frame;
 *     if (hex.readFrame(frame)) {
 *       Serial.println(frame.fx);
 *     }
 *   }
 * @endcode
 * 
 * Usage (software trigger mode):
 * @code
 *   hex.begin();
 *   hex.setTriggerMode(TriggerMode::SOFTWARE_TRIGGER);
 *   
 *   void loop() {
 *     hex.softwareTrigger();  // Send 'R' to request one sample
 *     delay(10);
 *     ResenseFrame frame;
 *     if (hex.readFrame(frame)) {
 *       Serial.println(frame.fx);
 *     }
 *   }
 * @endcode
 * 
 * Usage (hardware trigger mode):
 * @code
 *   hex.begin();
 *   hex.setTriggerMode(TriggerMode::HARDWARE_TRIGGER);  // Trigger pin = GPIO 5
 *   
 *   void loop() {
 *     hex.hardwareTrigger();  // Pulse GPIO 5
 *     delayMicroseconds(100);
 *     ResenseFrame frame;
 *     if (hex.readFrame(frame)) {
 *       Serial.println(frame.fx);
 *     }
 *   }
 * @endcode
 */
class ResenseHEX {
public:
  /**
   * @brief Constructor
   * @param serial Reference to a stream object (HardwareSerial, SoftwareSerial, etc.)
   */
  explicit ResenseHEX(Stream &serial);

 /**
   * @brief Initialize the library with default UART settings (2 Mbaud, 8-N-1)
   * 
   * Caller passes RX and TX pin numbers; baud and format use HEX defaults.
   * Call this after hardware is ready (typically in setup()).
   * 
   * @param rxPin RX pin number
   * @param txPin TX pin number
   * @return true if initialization was successful
   */
  bool begin(int rxPin, int txPin);

  /**
   * @brief Initialize with custom UART settings
   * 
   * @param baud Baud rate (e.g., 2000000 for 2 Mbaud)
   * @param config UART configuration (e.g., SERIAL_8N1)
   * @param rxPin RX pin number
   * @param txPin TX pin number
   * @return true if initialization was successful
   */
  bool begin(unsigned long baud, uint32_t config, int rxPin, int txPin);
  
  /**
   * @brief Attempt to read one complete frame from the sensor and converts it to a ResenseFrame struct
   * 
   * This function accumulates bytes from the serial stream and returns true
   * when a complete 28-byte frame has been received and decoded.
   * 
   * @param[out] frame Reference to ResenseFrame struct to populate
   * @return true if a complete frame was successfully read, false otherwise
   */
  bool readFrame(ResenseFrame &frame);
  
    /**
   * @brief Read one raw 28-byte frame into a user buffer
   *
   * This function fills the internal buffer from the serial stream and, when
   * at least FRAME_SIZE bytes are available, copies them verbatim into the
   * provided buffer (no decoding or validation).
   *
   * @param[out] frameData User-provided buffer of size >= FRAME_SIZE
   * @return true if 28 bytes were copied into frameData, false otherwise
   */
  bool readRawFrame(uint8_t *frameData);
  

  /**
   * @brief Check if at least one complete frame is likely available
   * 
   * @return true if serial buffer contains >= 28 bytes
   */
  bool availableFrame() const;

  /**
   * @brief Flush/discard all buffered data
   * 
   * Useful for resynchronization after connection loss or startup.
   */
  void flushInput();

  /**
   * @brief Attempt to resynchronize with the data stream
   * 
   * This function discards bytes one at a time until it finds what appears
   * to be a valid 28-byte aligned frame. Use this if you suspect frame
   * misalignment (e.g., after a disconnect/reconnect or glitch).
   * 
   * @param maxAttempts Maximum number of frames to check before giving up
   * @return true if resynchronization was successful
   */
  bool resync(uint16_t maxAttempts = 100);

  /**
   * @brief Get the number of bytes currently in the internal buffer
   * 
   * @return Byte count in buffer
   */
  size_t bufferedBytes() const;

  /**
   * @brief Get the number of successfully decoded frames since begin()
   * 
   * @return Frame count
   */
  uint32_t getFrameCount() const;

  /**
   * @brief Get the number of frames skipped due to alignment issues
   * 
   * @return Skip count
   */
  uint32_t getSkipCount() const;

  /**
   * @brief Set the trigger mode for data acquisition
   * 
   * @param mode The trigger mode (CONTINUOUS, SOFTWARE_TRIGGER, or HARDWARE_TRIGGER)
   * @return true if mode was set successfully
   */
  bool setTriggerMode(TriggerMode mode);

  /**
   * @brief Get the current trigger mode
   * 
   * @return Current TriggerMode
   */
  TriggerMode getTriggerMode() const;

  /**
   * @brief Send a software trigger (ASCII 'R') over UART
   * 
   * Only effective in SOFTWARE_TRIGGER mode. Requests one measurement sample
   * from the sensor.
   * 
   * @return true if trigger command was sent successfully
   */
  bool softwareTrigger();


  
private:
  static constexpr size_t FRAME_SIZE = 28;           ///< Size of one data frame in bytes
  static constexpr size_t BUFFER_SIZE = 256;         ///< Internal circular buffer size
  static constexpr float INVALID_FLOAT_THRESHOLD = 1e6f; ///< Threshold for detecting invalid floats
  static constexpr unsigned long DEFAULT_BAUD = 2000000UL; ///< HEX32 default: 2 Mbaud
  static constexpr uint32_t DEFAULT_CONFIG = SERIAL_8N1;   ///< HEX32 default: 8-N-1
  static constexpr char SOFTWARE_TRIGGER_CHAR = 'R';       ///< ASCII 'R' for software trigger
  static constexpr unsigned int HARDWARE_TRIGGER_PULSE_US = 100; ///< 100µs pulse width

  Stream &_serial;                           ///< Reference to serial stream
  uint8_t _buffer[BUFFER_SIZE];                      ///< Circular buffer for incoming bytes
  size_t _bufferHead;                                ///< Write position in buffer
  size_t _bufferTail;                                ///< Read position in buffer
  uint32_t _frameCount;                              ///< Count of successfully decoded frames
  uint32_t _skipCount;                               ///< Count of skipped/misaligned frames
  TriggerMode _triggerMode;                          ///< Current trigger mode

  /**
   * @brief Read bytes from serial into the internal buffer
   * 
   * @return Number of bytes added to buffer
   */
  size_t _fillBuffer();

  /**
   * @brief Check if internal buffer contains at least n bytes
   * 
   * @param n Minimum number of bytes to check for
   * @return true if buffer has at least n bytes available
   */
  bool _hasBytes(size_t n) const;

  /**
   * @brief Extract a frame-sized chunk from buffer and decode into ResenseFrame
   * 
   * Caller must ensure at least 28 bytes are in buffer.
   * 
   * @param[out] frame Reference to output frame struct
   * @return true if decoded successfully (basic validation passed)
   */
  bool _decodeFrame(ResenseFrame &frame);

  /**
   * @brief Perform basic sanity checks on decoded float values
   * 
   * @param frame The frame to validate
   * @return true if all values appear reasonable
   */
  bool _validateFrame(const ResenseFrame &frame) const;

  /**
   * @brief Discard one byte from the front of the buffer
   */
  void _discardByte();

  /**
   * @brief Extract one byte from buffer without removing it
   * 
   * @param offset Offset from buffer tail (0 = next byte to read)
   * @return The byte at that position
   */
  uint8_t _peekByte(size_t offset) const;

  /**
   * @brief Extract bytes into a local buffer for decoding
   * 
   * Caller must ensure buffer has space and serial has data.
   * 
   * @param[out] dest Destination buffer (must be >= 28 bytes)
   * @return Number of bytes extracted (should be 28)
   */
  size_t _extractFrame(uint8_t *dest);
};

#endif // RESENSE_HEX32_H