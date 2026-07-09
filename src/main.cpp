#include <Arduino.h>
#include "thijs_rplidar.h"   // Your custom header

// ----- Motor A (Right) -----
const int RPWM_RIGHT = 2;
const int LPWM_RIGHT = 4;

// ----- Motor B (Left) -----
const int RPWM_LEFT  = 18;
const int LPWM_LEFT  = 19;

// ----- RPLIDAR A1M8 -----
const int LIDAR_RX   = 32;
const int LIDAR_TX   = 33;
const int CTRL_MOTO  = 26;        // PWM pin to control lidar motor speed

// Create RPlidar instance using Serial2
RPlidar lidar(Serial2);

// ----- Lidar motor control (using analogWrite) -----
void setLidarMotor(uint8_t speed) {
    analogWrite(CTRL_MOTO, speed);
}

// ----- Data callback (prints at intervals to avoid overflow) -----
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // ms between prints

void lidarDataCallback(RPlidar* self, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
    // Skip invalid measurements (distance == 0)
    if (dist == 0) return;

    // Print only every PRINT_INTERVAL milliseconds
    unsigned long now = millis();
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = now;
        float angle_deg = angle_q6 / 64.0;   // q6 format -> degrees
        // Print angle and distance (CSV)
        Serial.print(angle_deg, 2);
        Serial.print(",");
        Serial.println(dist);
    }
}

// ----- Wheel motor sweep state (unchanged) -----
enum MotorState {
  SWEEP_RIGHT_FWD_LEFT_BACK,
  SWEEP_RIGHT_BACK_LEFT_FWD
};
MotorState state = SWEEP_RIGHT_FWD_LEFT_BACK;
int speedValue = 0;
int speedDirection = 1;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_UPDATE_INTERVAL = 50;  // ms

void setup() {
  // 1. Motor driver pins (PWM only)
  int motorPins[] = { RPWM_RIGHT, LPWM_RIGHT, RPWM_LEFT, LPWM_LEFT };
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // 2. Lidar motor control pin (PWM)
  pinMode(CTRL_MOTO, OUTPUT);
  setLidarMotor(0);   // start with motor off

  // 3. Serial communication
  Serial.begin(115200);                     // USB monitor
  // The library's init() will call Serial2.begin()
  lidar.init(LIDAR_RX, LIDAR_TX);           // set RX/TX pins and start serial

  // 4. Set the callback function
  lidar.postParseCallback = lidarDataCallback;

  delay(1000);
  Serial.println("=== RPLIDAR + Motor Test Started (thijs_rplidar) ===");
  Serial.println("Angle(deg), Distance(mm)");

  // 5. Check connection (as in example)
  if (!lidar.connectionCheck()) {
      Serial.println("connectionCheck() failed - check wiring!");
      while(1) { delay(100); }
  }
  Serial.println("LIDAR connected.");

  // Optional: print device info (as in example)
  // lidar.printLidarInfo();

  // 6. Start scanning – use Express Boost mode for more points (optional)
  // You can also use startStandardScan(false) if you prefer.
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
  if (!startSuccess) {
      Serial.println("Failed to start scan.");
      while(1) { delay(100); }
  }
  Serial.println("Scan started.");

  // 7. Spin the lidar motor – use a moderate speed (200 as in example)
  setLidarMotor(200);
}

void loop() {
  // ---- 1. Process lidar data (skip invalid, don't wait for checksum) ----
  int8_t result = lidar.handleData(false, false);   // parameters: includeInvalid, waitForChecksum
  if (result < 0) {
      Serial.println("LIDAR error – stopping motor.");
      setLidarMotor(0);
      while(1) { delay(100); }
  }

  // ---- 2. Update wheel motor PWM (non‑blocking sweep) ----
  unsigned long now = millis();
  if (now - lastSpeedUpdate >= SPEED_UPDATE_INTERVAL) {
    lastSpeedUpdate = now;

    speedValue += speedDirection;
    if (speedValue >= 255) {
      speedValue = 255;
      speedDirection = -1;
    } else if (speedValue <= 0) {
      speedValue = 0;
      speedDirection = 1;
      state = (state == SWEEP_RIGHT_FWD_LEFT_BACK) 
                ? SWEEP_RIGHT_BACK_LEFT_FWD 
                : SWEEP_RIGHT_FWD_LEFT_BACK;
    }

    switch (state) {
      case SWEEP_RIGHT_FWD_LEFT_BACK:
        analogWrite(RPWM_RIGHT, speedValue);
        analogWrite(LPWM_RIGHT, 0);
        analogWrite(RPWM_LEFT, 0);
        analogWrite(LPWM_LEFT, speedValue);
        break;
      case SWEEP_RIGHT_BACK_LEFT_FWD:
        analogWrite(RPWM_RIGHT, 0);
        analogWrite(LPWM_RIGHT, speedValue);
        analogWrite(RPWM_LEFT, speedValue);
        analogWrite(LPWM_LEFT, 0);
        break;
    }
  }
}