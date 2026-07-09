#include <Arduino.h>
#include "thijs_rplidar.h"

// ----- Motor pins -----
const int RPWM_RIGHT = 2;
const int LPWM_RIGHT = 4;
const int RPWM_LEFT  = 18;
const int LPWM_LEFT  = 19;

// ----- LIDAR -----
const int LIDAR_RX   = 32;
const int LIDAR_TX   = 33;
const int CTRL_MOTO  = 26;

// ----- IR sensors -----
const int RIGHT_IR = 34;
const int LEFT_IR  = 35;

RPlidar lidar(Serial2);

// ============================================================
// IR Wheel Angle Estimation (unchanged)
// ============================================================
const float TRANSITIONS_PER_REV = 7.0;
const float RAD_PER_TRANSITION = 2.0 * PI / TRANSITIONS_PER_REV;
const int IR_THRESHOLD = 2048;

float leftWheelAngle = 0.0;
float rightWheelAngle = 0.0;
int lastLeftIRState = -1;
int lastRightIRState = -1;

void updateWheelAngles() {
    int leftReading = analogRead(LEFT_IR);
    int rightReading = analogRead(RIGHT_IR);
    int leftState  = (leftReading  < IR_THRESHOLD) ? 0 : 1;
    int rightState = (rightReading < IR_THRESHOLD) ? 0 : 1;

    if (lastLeftIRState == 1 && leftState == 0)
        leftWheelAngle += RAD_PER_TRANSITION;
    if (lastRightIRState == 1 && rightState == 0)
        rightWheelAngle += RAD_PER_TRANSITION;

    lastLeftIRState = leftState;
    lastRightIRState = rightState;
}

// ============================================================
// NEW: Odometry Class (no STL dependencies)
// ============================================================
class Odometry {
private:
    double wheelRadius = 0.0975;   // meters
    double wheelBase = 0.33;       // meters (track width)
    double prevLeftPos = 0;
    double prevRightPos = 0;
    double prevTime = 0;
    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;

public:
    void setWheelRadius(double r) { wheelRadius = max(0.01, r); }   // uses Arduino macro
    void setWheelBase(double b)   { wheelBase = max(0.01, b); }

    void update(double leftPos, double rightPos, double currentTime) {
        if (firstUpdate) {
            prevLeftPos = leftPos;
            prevRightPos = rightPos;
            prevTime = currentTime;
            firstUpdate = false;
            return;
        }
        double dt = currentTime - prevTime;
        if (dt <= 0) return;
        double leftDist  = (leftPos - prevLeftPos) * wheelRadius;
        double rightDist = (rightPos - prevRightPos) * wheelRadius;
        linearVel  = (leftDist + rightDist) / (2.0 * dt);
        angularVel = (rightDist - leftDist) / (wheelBase * dt);
        double distance = (leftDist + rightDist) / 2.0;
        double deltaTheta = (rightDist - leftDist) / wheelBase;
        theta += deltaTheta;
        x += distance * cos(theta);
        y += distance * sin(theta);
        // Normalize theta to [-PI, PI]
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevTime = currentTime;
    }

    void printPose() {
        double thetaDeg = theta * 180.0 / M_PI;
        Serial.printf("Position: (%.3f m, %.3f m) | Theta: %.3f°\n", x, y, thetaDeg);
    }

    void reset() {
        x = y = theta = 0;
        linearVel = angularVel = 0;
        prevLeftPos = prevRightPos = 0;
        firstUpdate = true;
        Serial.println("[Odometry] Reset to origin");
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
    double getLinearVel() const { return linearVel; }
    double getAngularVel() const { return angularVel; }
};

// ============================================================
// Odometry instance and timing
// ============================================================
Odometry odom;
unsigned long lastOdomUpdate = 0;
const unsigned long ODOM_UPDATE_INTERVAL = 50;   // ms (approx 20 Hz)

// ============================================================
// Rest of original code (lidar, motor sweep, etc.)
// ============================================================

// Lidar motor control
void setLidarMotor(uint8_t speed) {
    analogWrite(CTRL_MOTO, speed);
}

// LiDAR callback (unchanged)
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;
void lidarDataCallback(RPlidar* self, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
    if (dist == 0) return;
    unsigned long now = millis();
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = now;
        float angle_deg = angle_q6 / 64.0;
        // Serial.print(angle_deg, 2); Serial.print(","); Serial.println(dist);
    }
}

// Motor sweep (unchanged)
enum MotorState {
  SWEEP_RIGHT_FWD_LEFT_BACK,
  SWEEP_RIGHT_BACK_LEFT_FWD
};
MotorState state = SWEEP_RIGHT_FWD_LEFT_BACK;
int speedValue = 0;
int speedDirection = 1;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_UPDATE_INTERVAL = 50;

// IR state print tracking (original)
int lastLeftState  = -1;
int lastRightState = -1;

// ============================================================
// setup()
// ============================================================
void setup() {
    // Motor pins
    int motorPins[] = { RPWM_RIGHT, LPWM_RIGHT, RPWM_LEFT, LPWM_LEFT };
    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i], OUTPUT);
        digitalWrite(motorPins[i], LOW);
    }
    pinMode(CTRL_MOTO, OUTPUT);
    setLidarMotor(0);

    Serial.begin(115200);
    lidar.init(LIDAR_RX, LIDAR_TX);
    lidar.postParseCallback = lidarDataCallback;

    delay(1000);
    Serial.println("=== Step 2: Odometry added ===");

    if (!lidar.connectionCheck()) {
        Serial.println("connectionCheck() failed!");
        while(1) delay(100);
    }
    Serial.println("LIDAR connected.");

    if (!lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST)) {
        Serial.println("Failed to start scan.");
        while(1) delay(100);
    }
    Serial.println("Scan started.");
    setLidarMotor(255);

    pinMode(RIGHT_IR, INPUT);
    pinMode(LEFT_IR, INPUT);

    // Initialise IR states
    lastLeftIRState = (analogRead(LEFT_IR) < IR_THRESHOLD) ? 0 : 1;
    lastRightIRState = (analogRead(RIGHT_IR) < IR_THRESHOLD) ? 0 : 1;

    Serial.println("Ready. Odometry pose printed every 2 seconds.");
}

// ============================================================
// loop()
// ============================================================
void loop() {
    // 1. LiDAR processing
    int8_t result = lidar.handleData(false, false);
    if (result < 0) {
        Serial.println("LIDAR error - stopping.");
        setLidarMotor(0);
        while(1) delay(100);
    }

    // 2. Motor sweep (original)
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

    // 3. Update wheel angles from IR (every loop)
    updateWheelAngles();

    // 4. Update odometry at fixed interval
    if (now - lastOdomUpdate >= ODOM_UPDATE_INTERVAL) {
        lastOdomUpdate = now;
        double currentTime = now / 1000.0;
        odom.update(leftWheelAngle, rightWheelAngle, currentTime);
    }

    // 5. Print pose every 2 seconds (for verification)
    static unsigned long lastPosePrint = 0;
    if (now - lastPosePrint >= 2000) {
        lastPosePrint = now;
        odom.printPose();
    }

    // 6. (Optional) Print IR state changes (original)
    int leftReading = analogRead(LEFT_IR);
    int rightReading = analogRead(RIGHT_IR);
    int leftState  = (leftReading  < IR_THRESHOLD) ? 0 : 1;
    int rightState = (rightReading < IR_THRESHOLD) ? 0 : 1;
    if (leftState != lastLeftState || rightState != lastRightState) {
        lastLeftState = leftState;
        lastRightState = rightState;
        Serial.printf("IR L:%d (state %d) R:%d (state %d) | L_ang=%.3f R_ang=%.3f\n",
                      leftReading, leftState, rightReading, rightState,
                      leftWheelAngle, rightWheelAngle);
    }
}