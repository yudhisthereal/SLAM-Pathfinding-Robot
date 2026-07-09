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

// ----- Infrared Sensors -----
const int RIGHT_IR = 34;
const int LEFT_IR  = 35;

// Create RPlidar instance using Serial2
RPlidar lidar(Serial2);

// ============================================================
// NEW: IR Wheel Angle Estimation
// ============================================================

// Wheel has 6 white stripes → 7 HIGH→LOW transitions per revolution
const float TRANSITIONS_PER_REV = 7.0;
const float RAD_PER_TRANSITION = 2.0 * PI / TRANSITIONS_PER_REV;

// Accumulated wheel angles (radians)
float leftWheelAngle = 0.0;
float rightWheelAngle = 0.0;

// Previous IR states for edge detection
int lastLeftIRState = -1;
int lastRightIRState = -1;

// Update wheel angles by counting HIGH→LOW transitions
void updateWheelAngles() {
    int leftReading = analogRead(LEFT_IR);
    int rightReading = analogRead(RIGHT_IR);
    // Threshold (from original main.cpp)
    int leftState  = (leftReading  < 2048) ? 0 : 1;
    int rightState = (rightReading < 2048) ? 0 : 1;

    // Left wheel: detect HIGH→LOW transition
    if (lastLeftIRState == 1 && leftState == 0) {
        leftWheelAngle += RAD_PER_TRANSITION;
    }
    lastLeftIRState = leftState;

    // Right wheel
    if (lastRightIRState == 1 && rightState == 0) {
        rightWheelAngle += RAD_PER_TRANSITION;
    }
    lastRightIRState = rightState;
}

// ============================================================
// Original code continues below
// ============================================================

// ----- Lidar motor control -----
void setLidarMotor(uint8_t speed) {
    analogWrite(CTRL_MOTO, speed);
}

// ----- Data callback (prints at intervals) -----
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // ms between prints

void lidarDataCallback(RPlidar* self, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
    if (dist == 0) return;
    unsigned long now = millis();
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = now;
        float angle_deg = angle_q6 / 64.0;
        // Uncomment to see LiDAR data
        // Serial.print(angle_deg, 2); Serial.print(","); Serial.println(dist);
    }
}

// ----- Wheel motor sweep state (original) -----
enum MotorState {
  SWEEP_RIGHT_FWD_LEFT_BACK,
  SWEEP_RIGHT_BACK_LEFT_FWD
};
MotorState state = SWEEP_RIGHT_FWD_LEFT_BACK;
int speedValue = 0;
int speedDirection = 1;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_UPDATE_INTERVAL = 50;  // ms

// ----- IR state tracking (original) -----
int lastLeftState  = -1;
int lastRightState = -1;

// ============================================================
// setup()
// ============================================================

void setup() {
  // 1. Motor driver pins
  int motorPins[] = { RPWM_RIGHT, LPWM_RIGHT, RPWM_LEFT, LPWM_LEFT };
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // 2. Lidar motor control pin
  pinMode(CTRL_MOTO, OUTPUT);
  setLidarMotor(0);

  // 3. Serial
  Serial.begin(115200);
  lidar.init(LIDAR_RX, LIDAR_TX);
  lidar.postParseCallback = lidarDataCallback;

  delay(1000);
  Serial.println("=== RPLIDAR + Motor Test Started (Step 1: IR Angle) ===");

  if (!lidar.connectionCheck()) {
      Serial.println("connectionCheck() failed - check wiring!");
      while(1) { delay(100); }
  }
  Serial.println("LIDAR connected.");

  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
  if (!startSuccess) {
      Serial.println("Failed to start scan.");
      while(1) { delay(100); }
  }
  Serial.println("Scan started.");
  setLidarMotor(255);

  // IR sensors
  pinMode(RIGHT_IR, INPUT);
  pinMode(LEFT_IR, INPUT);

  // Initialise IR states for angle estimation
  lastLeftIRState = (analogRead(LEFT_IR) < 26) ? 0 : 1;
  lastRightIRState = (analogRead(RIGHT_IR) < 26) ? 0 : 1;

  Serial.println("Ready. Wheel angles will be printed every 2 seconds.");
}

// ============================================================
// loop()
// ============================================================

void loop() {
  // ---- 1. Process lidar data ----
  int8_t result = lidar.handleData(false, false);
  if (result < 0) {
      Serial.println("LIDAR error - stopping motor.");
      setLidarMotor(0);
      while(1) { delay(100); }
  }

  // ---- 2. Update wheel motor PWM (original sweep) ----
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

  // ---- 3. Update wheel angles from IR (NEW) ----
  updateWheelAngles();

  // ---- 4. Read infrared sensors and print only on state change (original) ----
  int leftReading = analogRead(LEFT_IR);
  int rightReading = analogRead(RIGHT_IR);
  int leftState  = (leftReading  < 26) ? 0 : 1;
  int rightState = (rightReading < 26) ? 0 : 1;

  if (leftState != lastLeftState || rightState != lastRightState) {
      lastLeftState = leftState;
      lastRightState = rightState;
      Serial.print("IR Left: ");
      Serial.print(leftReading);
      Serial.print(" | IR Right: ");
      Serial.print(rightReading);
      // NEW: also print accumulated wheel angles
      Serial.print(" | Left angle: ");
      Serial.print(leftWheelAngle, 3);
      Serial.print(" rad | Right angle: ");
      Serial.println(rightWheelAngle, 3);
  }

  // Small yield to prevent watchdog issues
  // delay(1);
}