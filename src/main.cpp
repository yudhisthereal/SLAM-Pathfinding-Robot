#include <Arduino.h>

// Motor pins
const int RPWM_RIGHT = 12;
const int LPWM_RIGHT = 14;
const int RPWM_LEFT  = 32;
const int LPWM_LEFT  = 33;

// IR sensors
const int RIGHT_IR = 4;
const int LEFT_IR  = 2;

const float DEFAULT_MAX_SPEED = 4.0;
float maxSpeed = DEFAULT_MAX_SPEED;
const float PWM_PER_RAD_S = 255.0 / DEFAULT_MAX_SPEED;

const float TRANSITIONS_PER_REV = 13.0;
const float RAD_PER_TRANSITION = 2.0 * PI / TRANSITIONS_PER_REV;
const int IR_THRESHOLD = 2048;

float leftWheelAngle = 0.0, rightWheelAngle = 0.0;
int lastLeftIRState = -1, lastRightIRState = -1;

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

// ---- Odometry ----
class Odometry {
private:
    double wheelRadius = 0.0975;
    double wheelBase = 0.33;
    double prevLeftPos = 0, prevRightPos = 0;
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
    void reset() { x = y = theta = 0; linearVel = angularVel = 0; prevLeftPos = prevRightPos = 0; firstUpdate = true; }
    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
    double getLinearVel() const { return linearVel; }
    double getAngularVel() const { return angularVel; }
};

Odometry odom;

// ---- Velocity smoother ----
class VelocitySmoother {
private:
    double targetLeft = 0.0, targetRight = 0.0;
    double currentLeft = 0.0, currentRight = 0.0;
    double smoothingFactor = 0.15;
public:
    VelocitySmoother(double factor = 0.15) : smoothingFactor(factor) {}
    void setTarget(double left, double right) {
        targetLeft = left;
        targetRight = right;
    }
    void update(double dt) {
        if (dt > 0.1) dt = 0.032;
        currentLeft += (targetLeft - currentLeft) * smoothingFactor;
        currentRight += (targetRight - currentRight) * smoothingFactor;
        if (fabs(currentLeft) < 0.005) currentLeft = 0.0;
        if (fabs(currentRight) < 0.005) currentRight = 0.0;
    }
    double getLeftSpeed() const { return currentLeft; }
    double getRightSpeed() const { return currentRight; }
    void reset() { targetLeft = targetRight = currentLeft = currentRight = 0.0; }
};

// FIX: correct object declaration
VelocitySmoother smoother;   // default constructor with factor=0.15

// ---- Motor driver ----
void setMotorPWM(int rpwm, int lpwm, int speed) {
    if (speed >= 0) { analogWrite(rpwm, speed); analogWrite(lpwm, 0); }
    else { analogWrite(rpwm, 0); analogWrite(lpwm, -speed); }
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

// ---- State ----
bool autoMode = false;
bool pathActive = false;
bool pathExists = false;

#define MAX_WAYPOINTS 50
struct Waypoint { double x, y; };
Waypoint pathPoints[MAX_WAYPOINTS];
unsigned int numWaypoints = 0;
unsigned int pathIndex = 0;

float manualTargetLeft = 0.0, manualTargetRight = 0.0;

// ---- Command parser ----
#define CMD_BUFFER_SIZE 256
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;

void processCommand(const char* cmd) {
    if (strlen(cmd) == 0) return;

    if (strncmp(cmd, "MODE:", 5) == 0) {
        const char* mode = cmd + 5;
        if (strcmp(mode, "auto") == 0) {
            autoMode = true;
            pathActive = pathExists;
            if (pathActive) {
                pathIndex = 0;
                Serial.println("[Mode] Auto enabled");
            } else {
                Serial.println("[Mode] Auto enabled (no path)");
            }
        } else if (strcmp(mode, "manual") == 0 || strcmp(mode, "idle") == 0) {
            autoMode = false;
            pathActive = false;
            manualTargetLeft = 0.0;
            manualTargetRight = 0.0;
            smoother.setTarget(0.0, 0.0);
            setWheelSpeeds(0.0, 0.0);
            Serial.println("[Mode] Manual/Idle");
        } else {
            Serial.println("[Mode] Unknown");
        }
        return;
    }

    if (strncmp(cmd, "PATH:", 5) == 0) {
        numWaypoints = 0;
        pathExists = false;
        pathActive = false;
        const char* data = cmd + 5;
        char buffer[256];
        strncpy(buffer, data, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
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
            pathExists = true;
            Serial.printf("[Path] Stored %d waypoints\n", numWaypoints);
            if (autoMode) {
                pathActive = true;
                pathIndex = 0;
                Serial.println("[Path] Auto following started");
            }
        } else {
            pathExists = false;
            pathActive = false;
            Serial.println("[Path] Empty path");
        }
        return;
    }

    if (strncmp(cmd, "CONFIG:", 7) == 0) {
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
            Serial.printf("[Config] r=%.3f b=%.3f speed=%.2f\n", vals[0], vals[1], maxSpeed);
        } else {
            Serial.println("[Config] Invalid format");
        }
        return;
    }

    if (strncmp(cmd, "CMD:", 4) == 0) {
        const char* action = cmd + 4;
        if (!autoMode) {
            if (strcmp(action, "forward") == 0) {
                manualTargetLeft = maxSpeed;
                manualTargetRight = maxSpeed;
            } else if (strcmp(action, "backward") == 0) {
                manualTargetLeft = -maxSpeed;
                manualTargetRight = -maxSpeed;
            } else if (strcmp(action, "left") == 0) {
                manualTargetLeft = -maxSpeed;
                manualTargetRight = maxSpeed;
            } else if (strcmp(action, "right") == 0) {
                manualTargetLeft = maxSpeed;
                manualTargetRight = -maxSpeed;
            } else if (strcmp(action, "stop") == 0) {
                // Emergency stop – always works
                autoMode = false;
                pathActive = false;
                manualTargetLeft = 0.0;
                manualTargetRight = 0.0;
                smoother.setTarget(0.0, 0.0);
                setWheelSpeeds(0.0, 0.0);
                Serial.println("[CMD] Emergency stop (auto disabled)");
            } else {
                Serial.println("[CMD] Unknown");
            }
        } else {
            // Auto mode – only stop allowed
            if (strcmp(action, "stop") == 0) {
                autoMode = false;
                pathActive = false;
                manualTargetLeft = 0.0;
                manualTargetRight = 0.0;
                smoother.setTarget(0.0, 0.0);
                setWheelSpeeds(0.0, 0.0);
                Serial.println("[CMD] Emergency stop (auto disabled)");
            } else {
                Serial.println("[CMD] Ignored (auto mode)");
            }
        }
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        Serial.println("Commands: MODE:auto/manual/idle, PATH:x,y;..., CONFIG:r,b,speed,width,stop, CMD:forward/backward/left/right/stop");
        return;
    }
    if (strcmp(cmd, "reset") == 0) {
        odom.reset();
        smoother.reset();
        Serial.println("Odometry reset");
        return;
    }
    Serial.println("Unknown command");
}

// ---- Timing ----
unsigned long lastOdomUpdate = 0;
const unsigned long ODOM_UPDATE_INTERVAL = 50;
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_UPDATE_INTERVAL = 32;
float lastTime = 0.0;

void setup() {
    int motorPins[] = { RPWM_RIGHT, LPWM_RIGHT, RPWM_LEFT, LPWM_LEFT };
    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i], OUTPUT);
        digitalWrite(motorPins[i], LOW);
    }
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== ESP32 Robot (New Architecture) ===");
    pinMode(RIGHT_IR, INPUT);
    pinMode(LEFT_IR, INPUT);
    lastLeftIRState = (analogRead(LEFT_IR) < IR_THRESHOLD) ? 0 : 1;
    lastRightIRState = (analogRead(RIGHT_IR) < IR_THRESHOLD) ? 0 : 1;
    lastTime = millis() / 1000.0;
    Serial.println("Ready.");
}

void loop() {
    unsigned long now = millis();

    // ---- Serial commands ----
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
                Serial.println("Command too long");
            }
        }
    }

    // ---- Update IR ----
    updateWheelAngles();

    // ---- Control loop ----
    if (now - lastControlUpdate >= CONTROL_UPDATE_INTERVAL) {
        lastControlUpdate = now;
        float currentTime = now / 1000.0;
        float dt = currentTime - lastTime;
        lastTime = currentTime;

        float targetLeft = 0.0, targetRight = 0.0;

        if (autoMode && pathActive && pathIndex < numWaypoints) {
            double tx = pathPoints[pathIndex].x;
            double ty = pathPoints[pathIndex].y;
            double dxp = tx - odom.getX();
            double dyp = ty - odom.getY();
            double dist = sqrt(dxp*dxp + dyp*dyp);
            double desired = atan2(dyp, dxp);
            double diff = desired - odom.getTheta();
            while (diff > M_PI) diff -= 2*M_PI;
            while (diff < -M_PI) diff += 2*M_PI;

            const double turnThresh = 0.15;
            const double turnSpeed = maxSpeed * 0.35;
            bool isLast = (pathIndex == numWaypoints - 1);
            const double stopThreshold = 0.12;

            if (dist < stopThreshold) {
                if (isLast) {
                    Serial.println("[Path] Final goal reached");
                    pathActive = false;
                    targetLeft = targetRight = 0.0;
                } else {
                    pathIndex++;
                    Serial.printf("[Path] Waypoint %d/%d\n", pathIndex+1, numWaypoints);
                    targetLeft = targetRight = 0.0;
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
                if (isLast) {
                    double brakingZone = 1.2;
                    if (dist < brakingZone) {
                        speed *= (dist / brakingZone);
                        if (speed < 0.08) speed = 0.08;
                    }
                }
                targetLeft = speed;
                targetRight = speed;
            }
        } else {
            targetLeft = manualTargetLeft;
            targetRight = manualTargetRight;
        }

        smoother.setTarget(targetLeft, targetRight);
        smoother.update(dt);
        setWheelSpeeds(smoother.getLeftSpeed(), smoother.getRightSpeed());
    }

    // ---- Odometry and ODOM ----
    if (now - lastOdomUpdate >= ODOM_UPDATE_INTERVAL) {
        lastOdomUpdate = now;
        double currentTime = now / 1000.0;
        odom.update(leftWheelAngle, rightWheelAngle, currentTime);

        // Send ODOM without mode flag
        Serial.printf("ODOM,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            odom.getX(), odom.getY(), odom.getTheta(),
            smoother.getLeftSpeed(), smoother.getRightSpeed(),
            odom.getLinearVel(), odom.getAngularVel());
    }
}