/*!
 * @file DFRobot_64x8DTOF.cpp
 * @brief DFRobot_64x8DTOF class implementation
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (feng.yang@dfrobot.com)
 * @version V1.0.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */
#include "DFRobot_64x8DTOF.h"

DFRobot_64x8DTOF::DFRobot_64x8DTOF(HardwareSerial& serial, uint32_t config, int8_t rxPin, int8_t txPin)
{
  _serial = &serial;
  _config = config;
  _rxPin  = rxPin;
  _txPin  = txPin;
  _endPoint    = 0;
  _totalPoints = 0;
}

bool DFRobot_64x8DTOF::begin(uint32_t baudRate)
{
// Not supported platforms: ESP8266 and AVR (UNO/Mega etc.)
// Due to 921600 baud rate limitation, standard AVR 16MHz boards are not supported.
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_AVR)
  DBG("Error: platform not supported (ESP8266/AVR)");
  return false;
#endif

  if (baudRate != 921600) {
    DBG("Error: only 921600 baud rate is supported");
    return false;
  }

#if defined(ESP32)
  _serial->begin(baudRate, _config, _rxPin, _txPin);
#else
  // Fallback: try two-arg form first, otherwise single arg.
#if defined(HAVE_HWSERIAL1) || defined(SERIAL_8N1)
  _serial->begin(baudRate, _config);
#else
  _serial->begin(baudRate);
#endif
#endif

  delay(400);

  // Try to enable stream on the sensor(3 times); if it fails, device likely not present/responding
  bool isConnected = false;
  for (int i = 0; i < 3; i++) {
    if (setStreamControl(true)) {
      isConnected = true;
      break;
    }
    delay(100);
  }

  if (!isConnected) {
    DBG("Error:64x8DTOF not found or not responding");
    return false;
  }
  return true;
}

void DFRobot_64x8DTOF::clearBuffer(void)
{
  while (_serial->available()) _serial->read();
}

bool DFRobot_64x8DTOF::sendCommand(const String& command)
{
  clearBuffer();
  _serial->print(command);
  _serial->print("\n");

  uint32_t start      = millis();
  int      frameIndex = 0;
  while ((millis() - start) < DTOF64X8_RESPONSE_TIMEOUT) {
    if (_serial->available()) {
      uint8_t c = _serial->read();
      switch (frameIndex) {
        case 0:
          if (c == DTOF64X8_SYNC_BYTE_0)
            frameIndex++;
          break;
        case 1:
          if (c == DTOF64X8_SYNC_BYTE_1) {
            frameIndex++;
          } else {
            frameIndex = 0;
            if (c == DTOF64X8_SYNC_BYTE_0)
              frameIndex++;
          }
          break;
        case 2:
          if (c == DTOF64X8_SYNC_BYTE_2) {
            frameIndex++;
          } else {
            frameIndex = 0;
            if (c == DTOF64X8_SYNC_BYTE_0)
              frameIndex++;
          }
          break;
        case 3:
          if (c == DTOF64X8_SYNC_BYTE_3) {
            return true;
          } else {
            frameIndex = 0;
            if (c == DTOF64X8_SYNC_BYTE_0)
              frameIndex++;
          }
          break;
      }
    }
  }
  return false;
}

bool DFRobot_64x8DTOF::setStreamControl(bool enable)
{
  String command = "AT+STREAM_CONTROL=" + String(enable ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::setFrameMode(bool continuousMode)
{
  String command = "AT+SPAD_FRAME_MODE=" + String(continuousMode ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t endPoint)
{
  // Validat line range (0 allowed for global/full configuration)
  if (line > 8)
    return false;

  if (line != 0) {
    // For per-line configuration, enforce 1..64 indexing for points
    if (startPoint < 1 || startPoint > 64)
      return false;
    if (endPoint < 1 || endPoint > 64)
      return false;
    if (endPoint < startPoint)
      return false;
  }

  String command = "AT+SPAD_OUTPUT_LINE_DATA=" + String(line) + "," + String(startPoint) + "," + String(endPoint);
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::triggerOneFrame(void)
{
  String command = "AT+SPAD_TRIG_ONE_FRAME=1";
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::saveConfig(void)
{
  String command = "AT+SAVE_CONFIG";
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::configMeasureMode(uint8_t lineNum)
{
  if(lineNum <1 || lineNum > 8){
    DBG("Error: lineNum out of range");
    return false;
  }
  if (!setStreamControl(false))
    return false;
  delay(700);

  if (!setOutputLineData(lineNum, 1, 64))
    return false;
  _totalPoints = 64;
  DBG("Config: Single Line, Line: %d", lineNum);
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }

  delay(700);
  return setStreamControl(true);
}

bool DFRobot_64x8DTOF::configMeasureMode(uint8_t lineNum, uint8_t pointNum)
{
  if(lineNum <1 || lineNum > 8 || pointNum < 1 || pointNum > 64){
    DBG("Error: lineNum or pointNum out of range");
    return false;
  }
  if (!setStreamControl(false))
    return false;
  delay(700);

  if (!setOutputLineData(lineNum, pointNum, pointNum))
    return false;
  _totalPoints = 1;
  DBG("Config: Single Point, Line: %d, Point: %d", lineNum, pointNum);
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  return setStreamControl(true);
}

bool DFRobot_64x8DTOF::configMeasureMode(uint8_t lineNum, uint8_t startPoint, uint8_t endPoint)
{
  if(lineNum <1 || lineNum > 8 || startPoint < 1 || startPoint > 64 || endPoint < 1 || endPoint > 64 || endPoint < startPoint){
    DBG("Error: lineNum, startPoint or endPoint out of range");
    return false;
  }
  if (!setStreamControl(false))
    return false;
  delay(700);

  if (!setOutputLineData(lineNum, startPoint, endPoint))
    return false;
  _totalPoints = endPoint - startPoint + 1;
  DBG("Config: Multi Points, Line: %d, Start: %d, End: %d", lineNum, startPoint, endPoint);
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  return setStreamControl(true);
}

bool DFRobot_64x8DTOF::configMeasureMode(void)
{
  if (!setStreamControl(false))
    return false;
  delay(700);

  if (!setOutputLineData(0, 0, 0))
    return false;
  _totalPoints = DTOF64X8_MAX_POINTS;
  DBG("Config: Full Mode");
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  delay(700);
  return setStreamControl(true);
}

bool DFRobot_64x8DTOF::configFrameMode(eFrameMode_t mode)
{
  DBG("Configuring frame mode: %d", mode);

  if (!setStreamControl(false))
    return false;
  delay(700);
  bool frameMode = ((mode == eFrameSingle) ? true : false);

  if (!setFrameMode(frameMode))
    return false;
  delay(700);
  if (!saveConfig()) {
    DBG("Warning: saveConfig failed after configFrameMode");
  }
  delay(700);
  return setStreamControl(true);
}

void DFRobot_64x8DTOF::parsePointData(const uint8_t* pointData, int16_t* x, int16_t* y, int16_t* z, int16_t* i)
{
  *x = (int16_t)((pointData[1] << 8) | pointData[0]);
  *y = (int16_t)((pointData[3] << 8) | pointData[2]);
  *z = (int16_t)((pointData[5] << 8) | pointData[4]);
  *i = (int16_t)((pointData[7] << 8) | pointData[6]);
}

int DFRobot_64x8DTOF::getData(uint32_t timeoutMs)
{
  int points = _totalPoints;

  if (!sendCommand("AT+SPAD_TRIG_ONE_FRAME=1")) {
    return -1;
  }

  uint32_t start = millis();
  uint8_t  pointBuf[DTOF64X8_POINT_DATA_SIZE];

  for (int i = 0; i < points; i++) {
    int byteCount = 0;
    while (byteCount < DTOF64X8_POINT_DATA_SIZE) {
      if ((millis() - start) > timeoutMs)
        return -1;

      if (_serial->available()) {
        pointBuf[byteCount++] = _serial->read();
      }
    }

    parsePointData(pointBuf, &point.xBuf[i], &point.yBuf[i], &point.zBuf[i], &point.iBuf[i]);
  }

  return points;
}

