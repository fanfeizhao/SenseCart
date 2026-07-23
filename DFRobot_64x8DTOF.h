/*!
 * @file DFRobot_64x8DTOF.h
 * @brief DFRobot_64x8DTOF class infrastructure
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (feng.yang@dfrobot.com)
 * @version V1.0.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */
#ifndef _DFROBOT_64X8DTOF_H_
#define _DFROBOT_64X8DTOF_H_

#include <Arduino.h>
#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                \
  {                             \
    Serial.print("[");          \
    Serial.print(__FUNCTION__); \
    Serial.print(":");          \
    Serial.print(__LINE__);     \
    Serial.print(" ] ");        \
    char _buf[128];             \
    sprintf(_buf, __VA_ARGS__); \
    Serial.println(_buf);       \
  }
#else
#define DBG(...)
#endif

#define DTOF64X8_RESPONSE_TIMEOUT    500 /*!< Response timeout in milliseconds */
#define DTOF64X8_MAX_RETRY_COUNT     5   /*!< Maximum number of retries for commands */
#define DTOF64X8_RESPONSE_OK_SEQ_LEN 4   /*!< Length of OK response sequence */
#define DTOF64X8_SYNC_BYTE_0         0x0A
#define DTOF64X8_SYNC_BYTE_1         0x4F
#define DTOF64X8_SYNC_BYTE_2         0x4B
#define DTOF64X8_SYNC_BYTE_3         0x0A
#define DTOF64X8_FRAME_HEADER_SIZE   4
#define DTOF64X8_POINT_DATA_SIZE     8
#define DTOF64X8_MAX_POINTS          (64 * 8)

class DFRobot_64x8DTOF {
private:
  HardwareSerial* _serial; /*!< Hardware serial port */
  uint32_t        _config; /*!< Serial port configuration */
  int8_t          _rxPin;  /*!< RX pin number */
  int8_t          _txPin;  /*!< TX pin number */

  /**
   * @fn sendCommand
   * @brief Send AT command and wait for response
   * @param command AT command to send
   * @return Whether the command was sent successfully
   * @retval true: Success
   * @retval false: Failure
   */
  bool sendCommand(const String& command);

  /**
   * @fn setStreamControl
   * @brief Set stream control
   * @param enable Enable stream control
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setStreamControl(bool enable);

  /**
   * @fn setFrameMode
   * @brief Set frame mode
   * @param continuousMode Continuous mode flag
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setFrameMode(bool continuousMode);

  /**
   * @fn saveConfig
   * @brief Save configuration to sensor
   * @return Whether the operation was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool saveConfig(void);

  /**
   * @fn setOutputLineData
   * @brief Set output line data
   * @param line Line number
   * @param startPoint Start point number
   * @param endPoint End point number
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t endPoint);

  /**
   * @fn parsePointData
   * @brief Parse point data
   * @param pointData Point data to parse
   * @param x X-axis coordinate
   * @param y Y-axis coordinate
   * @param z Z-axis coordinate (Distance)
   * @param i Intensity
   */
  void parsePointData(const uint8_t* pointData, int16_t* x, int16_t* y, int16_t* z, int16_t* i);

  /**
   * @fn triggerOneFrame
   * @brief Trigger one frame data output
   * @return Whether the operation was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool triggerOneFrame(void);

  /**
   * @fn clearBuffer
   * @brief Clear serial receive buffer
   */
  void clearBuffer(void);

  int _startPoint;  /*!< Start point */
  int _endPoint;    /*!< End point */
  int _totalPoints; /*!< Total points */

public:
  /**
   * @enum FrameMode_t
   * @brief Frame mode enumeration: single frame / continuous frame
   */
  typedef enum {
    eFrameSingle     = 0, /*!< Single frame mode */
    eFrameContinuous = 1  /*!< Continuous frame mode */
  } eFrameMode_t;

  /**
   * @enum eMeasureMode_t
   * @brief Measurement output mode: single point / single line / full output
   */
  typedef enum {
    eMeasureModeSinglePoint = 0, /*!< Single point output (provide line and point) */
    eMeasureModeSingleLine  = 1, /*!< Single line output (provide line only) */
    eMeasureModeFull        = 2  /*!< Full output (all points) */
  } eMeasureMode_t;

  /**
   * @struct sPoint_t
   * @brief DFRobot 64x8DTOF sensor point data structure
   */
  typedef struct {
    int16_t xBuf[DTOF64X8_MAX_POINTS]; /*!< X-axis coordinate buffer */
    int16_t yBuf[DTOF64X8_MAX_POINTS]; /*!< Y-axis coordinate buffer */
    int16_t zBuf[DTOF64X8_MAX_POINTS]; /*!< Z-axis coordinate buffer (Distance) */
    int16_t iBuf[DTOF64X8_MAX_POINTS]; /*!< Intensity buffer */
  } sPoint_t;

  /**
  * @fn DFRobot_64x8DTOF
   * @brief Constructor, passing in serial port and configuration
   * @param serial Hardware serial port pointer
   * @param config Serial port configuration (e.g., SERIAL_8N1)
   * @param rxPin RX pin number
   * @param txPin TX pin number
   */
  DFRobot_64x8DTOF(HardwareSerial& serial = Serial1, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1);

  /**
   * @fn begin
    * @brief Initialize the sensor serial interface and enable data stream
    * @param baudRate Serial communication baud rate (must be 921600)
    * @return bool True if initialization succeeded (serial started and stream enabled), false on error
    * @note On ESP8266 and AVR platforms this function currently returns false (platform not supported).
    * @note The function will attempt to enable stream control on the device to verify presence.
   */
  bool begin(uint32_t baudRate = 921600);

  /**
   * @fn getPointData
   * @brief Trigger one frame and read raw x/y/z values (no filtering)
   * @param timeoutMs Timeout in milliseconds to wait for a complete frame
   * @return Number of points parsed, or -1 on error/timeout,-2 sizeout
   */
  int getData(uint32_t timeoutMs = 300);

  /**
   * @fn configMeasureMode
   * @brief Configure measurement output mode — Full output (all points).
   * @return bool True if configuration succeeded and stream control restored,
   *              false on communication error or device rejection.
   */
  bool configMeasureMode(void);

  /**
   * @fn configMeasureMode
   * @brief Configure measurement output mode — Single line.
   * @param lineNum Line index to output (1..8).
   * @return bool True if configuration succeeded and stream control restored,
   *              false on communication error or invalid arguments.
   */
  bool configMeasureMode(uint8_t lineNum);

  /**
   * @fn configMeasureMode
   * @brief Configure measurement output mode — Single point.
   * @param lineNum Line index containing the point (1..8).
   * @param pointNum Point index within the line (1..64).
   * @return bool True if configuration succeeded and stream control restored,
   *              false on communication error or invalid arguments.
   */
  bool configMeasureMode(uint8_t lineNum, uint8_t pointNum);

  /**
   * @fn configMeasureMode
   * @brief Configure measurement output mode — Multiple points within one line.
   * @param lineNum Line index containing the points (1..8).
   * @param startPoint Start point index within the line (1..64).
   * @param endPoint End point index within the line (1..64).
   * @return bool True if configuration succeeded and stream control restored,
   *              false on communication error or invalid arguments.
   */
  bool configMeasureMode(uint8_t lineNum, uint8_t startPoint, uint8_t endPoint);

  /**
   * @fn configFrameMode
   * @brief Configure frame mode (single frame or continuous)
   * @param mode Frame mode (eFrameSingle or eFrameContinuous)
   * @note Continuous mode is not implemented yet — do not use it.
   * @return bool type, indicates the configuration status
   * @retval true Configuration successful
   * @retval false Configuration failed
   */
  bool configFrameMode(eFrameMode_t mode);
  
  sPoint_t point; /*!< Point data */
};

#endif
