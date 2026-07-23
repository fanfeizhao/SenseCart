#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include "DFRobot_64x8DTOF.h"

#define TF_RX D10
#define TF_TX D11

#define SEN_RX D2
#define SEN_TX D3

#define XSHUT_L1X D8
#define XSHUT_L0X D9
#define XSHUT_L0X_EXTRA D12

#define LEFT_MOTOR D7
#define RIGHT_MOTOR D6

#define LED_GREEN D5   // right
#define LED_RED A2     // down / TF-Nova edge
#define LED_BLUE A1    // left
#define LED_YELLOW A3  // center

HardwareSerial tfSerial(2);
HardwareSerial SensorSerial(1);

DFRobot_64x8DTOF dtof64x8(SensorSerial, SERIAL_8N1, SEN_RX, SEN_TX);

VL53L0X leftL0X;
VL53L0X leftExtraL0X;
VL53L1X rightL1X;

#define LINE_NUM 4

const int MIN_VALID_MM = 350; //80 , /240
const int MAX_VALID_MM = 2500;

const int CENTER_START = 24;
const int CENTER_END = 39;

int senVals[64];

int centerDist = -1;
int leftDist = -1;        // combined left distance
int leftMainDist = -1;    // D9
int leftExtraDist = -1;   // D12
int rightDist = -1;
int downDist = -1;

const int CLIFF_THRESHOLD_MM = 220; //changed from 180 and 420 (rn 15 cm above ground + 70 mm buffer)

const int CENTER_CLOSE_MM = 1070; //1250, 1050
const int LEFT_CLOSE_MM = 1050; //900 , 700
const int RIGHT_CLOSE_MM = 1050; //900 , 700

const int MIN_PWM = 175;
const int MAX_PWM = 255;

const unsigned long SEN_INTERVAL = 450;
const unsigned long VL53_INTERVAL = 240;
const unsigned long TF_INTERVAL = 20;
const unsigned long PRINT_INTERVAL = 800;
const unsigned long EDGE_PULSE_INTERVAL = 120; //300
const unsigned long ALL_CLOSE_PULSE_INTERVAL = 120; //350

unsigned long lastSEN = 0;
unsigned long lastVL53 = 0;
unsigned long lastTF = 0;
unsigned long lastPrint = 0;
unsigned long lastPulse = 0;
unsigned long lastAllClosePulse = 0;
unsigned long lastTFReadingTime = 0;

bool edgeDetected = false;
bool pulseLeft = true;
bool allClosePulseOn = true;

int leftPWM = 0;
int rightPWM = 0;

String detected = "NONE";

bool leftOK = false;
bool leftExtraOK = false;
bool rightOK = false;

int badLeftCount = 0;
int badLeftExtraCount = 0;
int badRightCount = 0;

int speedFromDistance(int d) {
  if (d <= 0) return 0;
  d = constrain(d, 100, 1000); //constrain(d, 100, 1200);
  return map(d, 1000, 100, MIN_PWM, MAX_PWM); // map(d, 1200, 100, MIN_PWM, MAX_PWM);
}

int combineLeftSensors(int d1, int d2) {
  if (d1 > 0 && d2 > 0) return min(d1, d2);  // closest obstacle wins
  if (d1 > 0) return d1;
  if (d2 > 0) return d2;
  return -1;
}

/* void motorsOffBriefly() {
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  delay(5);
} */

int readTFNova() {
  uint8_t buf[9];
  int newestDist = -1;

  while (tfSerial.available() >= 9) {
    if (tfSerial.read() != 0x59) continue;
    if (tfSerial.read() != 0x59) continue;

    buf[0] = 0x59;
    buf[1] = 0x59;

    for (int i = 2; i < 9; i++) {
      buf[i] = tfSerial.read();
    }

    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += buf[i];

    if (sum != buf[8]) continue;

    int dist_cm = buf[2] | (buf[3] << 8);

    if (dist_cm >= 2 && dist_cm <= 500) {
      newestDist = dist_cm * 10;
    }
  }

  return newestDist;
}

int readSEN0682Center() {
  int count = dtof64x8.getData(120);

  if (count <= 0) return centerDist;

  int n = 0;

  for (int i = 0; i < count && i < 64; i++) {
    if (i < CENTER_START || i > CENTER_END) continue;

    int d = dtof64x8.point.zBuf[i];

    if (d >= MIN_VALID_MM && d <= MAX_VALID_MM) {
      senVals[n++] = d;
    }
  }

  if (n < 4) return -1; //if (n == 0) return centerDist;

  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (senVals[j] < senVals[i]) {
        int temp = senVals[i];
        senVals[i] = senVals[j];
        senVals[j] = temp;
      }
    }
  }

  int useCount = n > 4 ? 4 : n;
  long sum = 0;

  for (int i = 0; i < useCount; i++) sum += senVals[i];

  return sum / useCount;
}

void setupVL53() {
  pinMode(XSHUT_L1X, OUTPUT);
  pinMode(XSHUT_L0X, OUTPUT);
  pinMode(XSHUT_L0X_EXTRA, OUTPUT);

  digitalWrite(XSHUT_L1X, LOW);
  digitalWrite(XSHUT_L0X, LOW);
  digitalWrite(XSHUT_L0X_EXTRA, LOW);
  delay(200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  Wire.begin(A4, A5);
  Wire.setClock(100000);
  delay(150);

  // LEFT MAIN VL53L0X D9 -> 0x30
  digitalWrite(XSHUT_L0X, HIGH);
  delay(200);

  leftL0X.setTimeout(300);

  if (!leftL0X.init()) {
    Serial.println("VL53L0X LEFT MAIN D9 failed");
    leftOK = false;
  } else {
    leftL0X.setAddress(0x30);
    delay(80);
    leftL0X.setMeasurementTimingBudget(50000);
    leftL0X.startContinuous(80);
    leftOK = true;
    Serial.println("VL53L0X LEFT MAIN D9 OK at 0x30");
  }

  delay(150);

  // LEFT EXTRA VL53L0X D12 -> 0x31
  digitalWrite(XSHUT_L0X_EXTRA, HIGH);
  delay(200);

  leftExtraL0X.setTimeout(300);

  if (!leftExtraL0X.init()) {
    Serial.println("VL53L0X LEFT EXTRA D12 failed");
    leftExtraOK = false;
  } else {
    leftExtraL0X.setAddress(0x31);
    delay(80);
    leftExtraL0X.setMeasurementTimingBudget(50000);
    leftExtraL0X.startContinuous(80);
    leftExtraOK = true;
    Serial.println("VL53L0X LEFT EXTRA D12 OK at 0x31");
  }

  delay(150);

  // RIGHT VL53L1X D8 -> 0x32
  digitalWrite(XSHUT_L1X, HIGH);
  delay(200);

  rightL1X.setTimeout(300);

  if (!rightL1X.init()) {
    Serial.println("VL53L1X RIGHT D8 failed");
    rightOK = false;
  } else {
    rightL1X.setAddress(0x32);
    delay(80);
    rightL1X.setDistanceMode(VL53L1X::Long);
    rightL1X.setMeasurementTimingBudget(50000);
    rightL1X.startContinuous(80);
    rightOK = true;
    Serial.println("VL53L1X RIGHT D8 OK at 0x32");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting STABLE cart system with 2 LEFT VL53L0X...");

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);

  tfSerial.begin(115200, SERIAL_8N1, TF_RX, TF_TX);
  Serial.println("TF-Nova UART started");

  setupVL53();

  Serial.println("Starting SEN0682...");

  while (!dtof64x8.begin()) {
    Serial.println("SEN0682 begin failed...");
    delay(300);
  }

  while (!dtof64x8.configFrameMode(DFRobot_64x8DTOF::eFrameSingle)) {
    Serial.println("SEN0682 frame mode failed...");
    delay(300);
  }

  while (!dtof64x8.configMeasureMode(LINE_NUM)) {
    Serial.println("SEN0682 line mode failed...");
    delay(200);
  }

  Serial.println("SEN0682 OK");
  Serial.println("System ready");
}

void loop() {
  if (millis() - lastTF >= TF_INTERVAL) {
    lastTF = millis();

    int newDown = readTFNova();

    if (newDown > 0) {
      downDist = newDown;
      lastTFReadingTime = millis();
    }

    if (millis() - lastTFReadingTime > 180) {
      edgeDetected = false;
      downDist = -1;
    } else {
      edgeDetected = downDist > CLIFF_THRESHOLD_MM;
    }
  }

  if (millis() - lastVL53 >= VL53_INTERVAL) {
    lastVL53 = millis();

    // motorsOffBriefly();

    if (leftOK) {
      int newLeft = leftL0X.readRangeContinuousMillimeters();

      if (leftL0X.timeoutOccurred() || newLeft <= 0 || newLeft > 3000) {
        badLeftCount++;
        if (badLeftCount >= 3) leftMainDist = -1;
      } else {
        badLeftCount = 0;
        leftMainDist = newLeft;
      }
    }

    delay(35);

    if (leftExtraOK) {
      int newLeftExtra = leftExtraL0X.readRangeContinuousMillimeters();

      if (leftExtraL0X.timeoutOccurred() || newLeftExtra <= 0 || newLeftExtra > 3000) {
        badLeftExtraCount++;
        if (badLeftExtraCount >= 3) leftExtraDist = -1;
      } else {
        badLeftExtraCount = 0;
        leftExtraDist = newLeftExtra;
      }
    }

    leftDist = combineLeftSensors(leftMainDist, leftExtraDist);

    delay(35);

    if (rightOK) {
      int newRight = rightL1X.read();

      if (rightL1X.timeoutOccurred() || newRight <= 0 || newRight > 3000) {
        badRightCount++;
        if (badRightCount >= 3) rightDist = -1;
      } else {
        badRightCount = 0;
        rightDist = newRight;
      }
    }

    delay(35);
  }

  if (millis() - lastSEN >= SEN_INTERVAL) {
    lastSEN = millis();

    int newCenter = readSEN0682Center();

    if (newCenter > 0) {
      centerDist = newCenter;
    }
  }

  bool centerClose = centerDist > 0 && centerDist < CENTER_CLOSE_MM;
  bool leftClose = leftDist > 0 && leftDist < LEFT_CLOSE_MM;
  bool rightClose = rightDist > 0 && rightDist < RIGHT_CLOSE_MM;
  bool allClose = centerClose && leftClose && rightClose;

  digitalWrite(LED_BLUE, leftClose ? HIGH : LOW);
  digitalWrite(LED_YELLOW, centerClose ? HIGH : LOW);
  digitalWrite(LED_GREEN, rightClose ? HIGH : LOW);
  digitalWrite(LED_RED, edgeDetected ? HIGH : LOW);

  detected = "NONE";
  leftPWM = 0;
  rightPWM = 0;

  if (edgeDetected) {
    detected = "DOWN / EDGE";

    if (millis() - lastPulse >= EDGE_PULSE_INTERVAL) {
      lastPulse = millis();
      pulseLeft = !pulseLeft;
    }

    if (pulseLeft) {
      leftPWM = 230;
      rightPWM = 0;
    } else {
      leftPWM = 0;
      rightPWM = 230;
    }
  } 
  else if (allClose) {
    detected = "ALL CLOSE";

    if (millis() - lastAllClosePulse >= ALL_CLOSE_PULSE_INTERVAL) {
      lastAllClosePulse = millis();
      allClosePulseOn = !allClosePulseOn;
    }

    if (allClosePulseOn) {
      leftPWM = 230;
      rightPWM = 230;
    } else {
      leftPWM = 0;
      rightPWM = 0;
    }
  } 
  else {
    int closestDist = 99999;

    if (centerDist > 0 && centerDist < closestDist) {
      closestDist = centerDist;
      detected = "CENTER";
      int pwm = speedFromDistance(centerDist);
      leftPWM = pwm;
      rightPWM = pwm;
    }

    if (leftDist > 0 && leftDist < closestDist) {
      closestDist = leftDist;
      detected = "LEFT";
      leftPWM = speedFromDistance(leftDist);
      rightPWM = 0;
    }

    if (rightDist > 0 && rightDist < closestDist) {
      closestDist = rightDist;
      detected = "RIGHT";
      leftPWM = 0;
      rightPWM = speedFromDistance(rightDist);
    }
  }

  analogWrite(LEFT_MOTOR, leftPWM);
  analogWrite(RIGHT_MOTOR, rightPWM);

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    Serial.println("-----");

    Serial.print("CENTER SEN0682: ");
    Serial.print(centerDist);
    Serial.println(" mm");

    Serial.print("LEFT MAIN L0X D9: ");
    Serial.print(leftMainDist);
    Serial.print(" mm, LEFT EXTRA L0X D12: ");
    Serial.print(leftExtraDist);
    Serial.print(" mm, COMBINED LEFT: ");
    Serial.print(leftDist);
    Serial.println(" mm");

    Serial.print("RIGHT VL53L1X: ");
    Serial.print(rightDist);
    Serial.println(" mm");

    Serial.print("DOWN TF-Nova: ");
    Serial.print(downDist);
    Serial.println(" mm");

    Serial.print("Detected: ");
    Serial.println(detected);

    Serial.print("PWM: left=");
    Serial.print(leftPWM);
    Serial.print(", right=");
    Serial.println(rightPWM);

    Serial.print("Bad counts: L1=");
    Serial.print(badLeftCount);
    Serial.print(" L2=");
    Serial.print(badLeftExtraCount);
    Serial.print(" R=");
    Serial.println(badRightCount);
  }
}