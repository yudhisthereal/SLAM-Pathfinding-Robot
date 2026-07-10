#include <Arduino.h>

// ----- Motor pins -----
const int RPWM_RIGHT = 2;
const int LPWM_RIGHT = 4;
const int RPWM_LEFT  = 18;
const int LPWM_LEFT  = 19;

// ----- IR sensors -----
const int RIGHT_IR = 34;
const int LEFT_IR  = 35;

// ============================================================
// Constants for motor control
// ============================================================
const float DEFAULT_MAX_SPEED = 4.0;    // rad/s
float maxSpeed = DEFAULT_MAX_SPEED;     // can be changed via CONFIG
const float PWM_PER_RAD_S = 255.0 / DEFAULT_MAX_SPEED;  // fixed mapping

// ============================================================
// IR Wheel Angle Estimation
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
// Odometry Class
// ============================================================
class Odometry {
private:
    double wheelRadius = 0.0975;
    double wheelBase = 0.33;
    double prevLeftPos = 0;
    double prevRightPos = 0;
    double prevTime = 0;
    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;

public:
    void setWheelRadius(double r) { wheelRadius = max(0.01, r); }
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

Odometry odom;

// ============================================================
// VelocitySmoother Class
// ============================================================
class VelocitySmoother {
private:
    double targetLeftSpeed = 0.0;
    double targetRightSpeed = 0.0;
    double currentLeftSpeed = 0.0;
    double currentRightSpeed = 0.0;
    double smoothingFactor = 0.15;

public:
    VelocitySmoother(double factor = 0.15) : smoothingFactor(factor) {}

    void setTarget(double left, double right) {
        targetLeftSpeed = left;
        targetRightSpeed = right;
    }

    void update(double dt) {
        if (dt > 0.1) dt = 0.032;
        currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * smoothingFactor;
        currentRightSpeed += (targetRightSpeed - currentRightSpeed) * smoothingFactor;
        if (fabs(currentLeftSpeed) < 0.005) currentLeftSpeed = 0.0;
        if (fabs(currentRightSpeed) < 0.005) currentRightSpeed = 0.0;
    }

    double getLeftSpeed() const { return currentLeftSpeed; }
    double getRightSpeed() const { return currentRightSpeed; }

    void setSmoothingFactor(double f) {
        smoothingFactor = max(0.05, min(0.5, f));
    }

    bool isMoving() const {
        return (fabs(currentLeftSpeed) > 0.01 || fabs(currentRightSpeed) > 0.01);
    }

    void reset() {
        targetLeftSpeed = targetRightSpeed = 0.0;
        currentLeftSpeed = currentRightSpeed = 0.0;
    }
};

VelocitySmoother smoother(0.15);

// ============================================================
// Motor driver functions
// ============================================================
void setMotorPWM(int rpwm, int lpwm, int speed) {
    if (speed >= 0) {
        analogWrite(rpwm, speed);
        analogWrite(lpwm, 0);
    } else {
        analogWrite(rpwm, 0);
        analogWrite(lpwm, -speed);
    }
}

void setWheelSpeeds(float leftRadPerSec, float rightRadPerSec) {
    leftRadPerSec  = constrain(leftRadPerSec,  -maxSpeed, maxSpeed);
    rightRadPerSec = constrain(rightRadPerSec, -maxSpeed, maxSpeed);
    int leftPWM  = (int)round(leftRadPerSec  * PWM_PER_RAD_S);
    int rightPWM = (int)round(rightRadPerSec * PWM_PER_RAD_S);
    leftPWM  = constrain(leftPWM,  -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);
    setMotorPWM(RPWM_LEFT, LPWM_LEFT, leftPWM);
    setMotorPWM(RPWM_RIGHT, LPWM_RIGHT, rightPWM);
}

// ============================================================
// Target speeds (manual commands)
// ============================================================
float manualTargetLeft = 0.0;
float manualTargetRight = 0.0;

// ============================================================
// Auto‑navigation state
// ============================================================
bool autoMode = false;
bool pathActive = false;
unsigned int pathIndex = 0;

#define MAX_WAYPOINTS 50
struct Waypoint { double x, y; };
Waypoint pathPoints[MAX_WAYPOINTS];
unsigned int numWaypoints = 0;

float robotWidth = 0.41;
float obstacleDistThres = 0.3;
bool obstacleAhead = false;

// ============================================================
// FULL command parser (matches Webots protocol)
// ============================================================
#define CMD_BUFFER_SIZE 256
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;

void processCommand(const char* cmd) {
    if (strlen(cmd) == 0) return;

    // Make a mutable copy for strtok
    char cmdCopy[CMD_BUFFER_SIZE];
    strncpy(cmdCopy, cmd, CMD_BUFFER_SIZE - 1);
    cmdCopy[CMD_BUFFER_SIZE - 1] = '\0';

    // ---- CMD: messages ----
    if (strncmp(cmd, "CMD:", 4) == 0) {
        const char* action = cmd + 4;
        // Only process manual commands if autoMode is OFF (matches simulation)
        if (!autoMode) {
            if (strcmp(action, "forward") == 0) {
                manualTargetLeft = maxSpeed;
                manualTargetRight = maxSpeed;
                Serial.println("[CMD] Forward");
            } else if (strcmp(action, "backward") == 0) {
                manualTargetLeft = -maxSpeed;
                manualTargetRight = -maxSpeed;
                Serial.println("[CMD] Backward");
            } else if (strcmp(action, "left") == 0) {
                manualTargetLeft = -maxSpeed;
                manualTargetRight = maxSpeed;
                Serial.println("[CMD] Turn left");
            } else if (strcmp(action, "right") == 0) {
                manualTargetLeft = maxSpeed;
                manualTargetRight = -maxSpeed;
                Serial.println("[CMD] Turn right");
            } else if (strcmp(action, "stop") == 0) {
                manualTargetLeft = 0.0;
                manualTargetRight = 0.0;
                Serial.println("[CMD] Stop");
            } else if (strcmp(action, "auto") == 0) {
                // Toggle auto mode
                autoMode = !autoMode;
                if (autoMode) {
                    Serial.println("[Auto] Enabled");
                    pathActive = (numWaypoints > 0);
                    pathIndex = 0;
                } else {
                    Serial.println("[Auto] Disabled");
                    pathActive = false;
                    pathIndex = 0;
                    manualTargetLeft = 0.0;
                    manualTargetRight = 0.0;
                    smoother.setTarget(0.0, 0.0);
                }
            } else {
                Serial.println("Unknown CMD");
            }
        } else {
            // autoMode is ON – ignore all manual commands including "auto"? 
            // (CMD:auto is already handled above when !autoMode; here it's ignored)
            Serial.println("Ignored – auto mode active");
        }
    }
    // ---- AUTO: messages ----
    else if (strncmp(cmd, "AUTO:", 5) == 0) {
        bool newAuto = (cmd[5] == '1');
        if (newAuto != autoMode) {
            autoMode = newAuto;
            if (autoMode) {
                Serial.println("[Auto] Enabled by bridge");
                pathActive = (numWaypoints > 0);
                pathIndex = 0;
            } else {
                Serial.println("[Auto] Disabled by bridge");
                pathActive = false;
                pathIndex = 0;
                manualTargetLeft = 0.0;
                manualTargetRight = 0.0;
                smoother.setTarget(0.0, 0.0);
            }
        }
    }
    // ---- CONFIG: messages ----
    else if (strncmp(cmd, "CONFIG:", 7) == 0) {
        char buffer[256];
        strncpy(buffer, cmd + 7, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
        char* token = strtok(buffer, ",");
        int idx = 0;
        float vals[5];
        while (token && idx < 5) {
            vals[idx++] = atof(token);
            token = strtok(NULL, ",");
        }
        if (idx == 5) {
            odom.setWheelRadius(max(0.01f, vals[0]));
            odom.setWheelBase(max(0.01f, vals[1]));
            maxSpeed = max(0.1f, min(10.0f, vals[2]));
            robotWidth = max(0.01f, vals[3]);
            obstacleDistThres = max(0.01f, vals[4]);
            Serial.printf("[Config] radius=%.3f base=%.3f maxSpeed=%.2f width=%.3f stopDist=%.3f\n",
                          vals[0], vals[1], maxSpeed, robotWidth, obstacleDistThres);
        } else {
            Serial.println("[Config] Invalid format, need 5 values");
        }
    }
    // ---- PATH: messages ----
    else if (strncmp(cmd, "PATH:", 5) == 0) {
        char buffer[256];
        strncpy(buffer, cmd + 5, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
        numWaypoints = 0;
        char* pair = strtok(buffer, ";");
        while (pair && numWaypoints < MAX_WAYPOINTS) {
            char* comma = strchr(pair, ',');
            if (comma) {
                *comma = '\0';
                double x = atof(pair);
                double y = atof(comma + 1);
                pathPoints[numWaypoints].x = x;
                pathPoints[numWaypoints].y = y;
                numWaypoints++;
            }
            pair = strtok(NULL, ";");
        }
        if (numWaypoints > 0) {
            pathActive = true;
            pathIndex = 0;
            Serial.printf("[Path] Received %d waypoints\n", numWaypoints);
        } else {
            pathActive = false;
            Serial.println("[Path] No valid waypoints");
        }
    }
    // ---- Local debug commands ----
    else if (strcmp(cmd, "help") == 0) {
        Serial.println("Commands (simulation protocol):");
        Serial.println("  CMD:forward/backward/left/right/stop/auto");
        Serial.println("  AUTO:1  or  AUTO:0");
        Serial.println("  CONFIG:radius,base,maxSpeed,width,stopDist");
        Serial.println("  PATH:x1,y1;x2,y2;...");
        Serial.println("Extra local: reset  (reset odometry)");
    } else if (strcmp(cmd, "reset") == 0) {
        odom.reset();
        smoother.reset();
        Serial.println("Odometry reset.");
    } else {
        Serial.println("Unknown command. Type 'help'.");
    }
}

// ============================================================
// Timing variables
// ============================================================
unsigned long lastOdomUpdate = 0;
const unsigned long ODOM_UPDATE_INTERVAL = 50;
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_UPDATE_INTERVAL = 32;
unsigned long lastObstacleCheck = 0;
const unsigned long OBSTACLE_CHECK_INTERVAL = 100;

float lastTime = 0.0;

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

    Serial.begin(115200);

    delay(1000);
    Serial.println("=== Robot Control - No LiDAR ===");

    pinMode(RIGHT_IR, INPUT);
    pinMode(LEFT_IR, INPUT);

    lastLeftIRState = (analogRead(LEFT_IR) < IR_THRESHOLD) ? 0 : 1;
    lastRightIRState = (analogRead(RIGHT_IR) < IR_THRESHOLD) ? 0 : 1;

    lastTime = millis() / 1000.0;
    Serial.println("Ready. Type 'help' for commands.");
}

// ============================================================
// loop()
// ============================================================
void loop() {
    unsigned long now = millis();

    // ---- 1. Process serial commands ----
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        } else if (c != '\r') {
            if (cmdIndex < CMD_BUFFER_SIZE - 1) {
                cmdBuffer[cmdIndex++] = c;
            } else {
                cmdIndex = 0;
                Serial.println("Command too long, ignored.");
            }
        }
    }

    // ---- 2. Update wheel angles ----
    updateWheelAngles();

    // ---- 3. Obstacle detection simulation (no LiDAR) ----
    if (now - lastObstacleCheck >= OBSTACLE_CHECK_INTERVAL) {
        lastObstacleCheck = now;
        // For demo purposes, obstacle detection is disabled
        // Could be replaced with ultrasonic or other sensors
        obstacleAhead = false;
    }

    // ---- 4. Control loop (navigation + motor commands) ----
    if (now - lastControlUpdate >= CONTROL_UPDATE_INTERVAL) {
        lastControlUpdate = now;
        float currentTime = now / 1000.0;
        float dt = currentTime - lastTime;
        lastTime = currentTime;

        float targetLeft = 0.0, targetRight = 0.0;

        if (obstacleAhead) {
            // Obstacle detected – stop
            targetLeft = 0.0;
            targetRight = 0.0;
        } else if (autoMode && pathActive && pathIndex < numWaypoints) {
            // ---- Autonomous navigation ----
            double tx = pathPoints[pathIndex].x;
            double ty = pathPoints[pathIndex].y;
            double dxp = tx - odom.getX();
            double dyp = ty - odom.getY();
            double dist = sqrt(dxp*dxp + dyp*dyp);
            double desired_heading = atan2(dyp, dxp);
            double diff = desired_heading - odom.getTheta();
            while (diff > M_PI) diff -= 2*M_PI;
            while (diff < -M_PI) diff += 2*M_PI;

            const double turnThresh = 0.15;
            const double turnSpeed = maxSpeed * 0.35;
            bool isLastWaypoint = (pathIndex == numWaypoints - 1);
            const double stopThreshold = 0.12;

            if (dist < stopThreshold) {
                if (isLastWaypoint) {
                    Serial.println("[Path] Final goal reached! Stopping.");
                    pathActive = false;
                    pathIndex = 0;
                    targetLeft = 0.0;
                    targetRight = 0.0;
                } else {
                    pathIndex++;
                    Serial.printf("[Path] Reached waypoint %d/%d\n", pathIndex, numWaypoints);
                    targetLeft = 0.0;
                    targetRight = 0.0;
                }
            } else if (fabs(diff) > turnThresh) {
                if (diff > 0) {
                    targetLeft = turnSpeed * 0.05f;
                    targetRight = turnSpeed;
                } else {
                    targetLeft = turnSpeed;
                    targetRight = turnSpeed * 0.05f;
                }
            } else {
                double speed = maxSpeed * 0.9;
                if (isLastWaypoint) {
                    double brakingZone = 1.2;
                    if (dist < brakingZone) {
                        speed = speed * (dist / brakingZone);
                        if (speed < 0.08) speed = 0.08;
                    }
                }
                targetLeft = speed;
                targetRight = speed;
            }
        } else {
            // Manual mode
            targetLeft = manualTargetLeft;
            targetRight = manualTargetRight;
        }

        // Apply to smoother
        smoother.setTarget(targetLeft, targetRight);
        smoother.update(dt);
        setWheelSpeeds(smoother.getLeftSpeed(), smoother.getRightSpeed());
    }

    // ---- 5. Update odometry ----
    if (now - lastOdomUpdate >= ODOM_UPDATE_INTERVAL) {
        lastOdomUpdate = now;
        double currentTime = now / 1000.0;
        odom.update(leftWheelAngle, rightWheelAngle, currentTime);
    }

    // ---- 6. Print pose every 2 seconds ----
    static unsigned long lastPosePrint = 0;
    if (now - lastPosePrint >= 2000) {
        lastPosePrint = now;
        odom.printPose();
    }
}