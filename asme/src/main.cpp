#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>
#include <N20motor.h>
#include <MG996R_angle.h>
#include <ESP32Servo.h>

BleXboxController ble;
MotorControl ServoA(18);
MotorControl ServoY(19);
MotorControl ServoX(21);
MotorControl ServoB(22);
MG996R_angle mg996r(23);  // ★ MG996R

// ★ 4個servo角度變數
float servoAngleA = 90.0f;
float servoAngleY = 90.0f;
float servoAngleX = 90.0f;
float servoAngleB = 90.0f;

// ★ MG996R 新邏輯變數：按一下LB正轉、按一下RB反轉、LT立即停止
static bool mgLB_Toggle = false;      // LB 按鈕切換狀態
static bool mgRB_Toggle = false;      // RB 按鈕切換狀態
static float mgAngle = 90.0f;         // 單一角度變數
static int mgDirection = 0;           // 0=停, 1=正轉, -1=反轉
static unsigned long mgLastUpdate = 0;
static const unsigned long MG_SPEED_DELAY = 50;  // 轉速延遲(ms)

// ★ 原有4個servo變數
int servoDirectionA = 1, servoDirectionY = 1, servoDirectionX = 1, servoDirectionB = 1;
bool wasPressedA = false, wasPressedY = false, wasPressedX = false, wasPressedB = false;
bool wasLB = false, wasRB = false;  // 用於偵測按下瞬間
const float servoSpeed = 4.0f;

#define N20_ENA 12
#define N20_IN1 14
#define N20_IN2 27
#define N20_ENB 33
#define N20_IN3 26
#define N20_IN4 25
#define DEADZONE 0.15f
#define MAX_PWM 200

void setupN20Motor() {
    pinMode(N20_ENA, OUTPUT); pinMode(N20_IN1, OUTPUT); pinMode(N20_IN2, OUTPUT);
    pinMode(N20_ENB, OUTPUT); pinMode(N20_IN3, OUTPUT); pinMode(N20_IN4, OUTPUT);
    analogWrite(N20_ENA, 0); analogWrite(N20_ENB, 0);
}

void setMotor(int speed, int pinEN, int pinINA, int pinINB) {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);
    if (speed > 0) {
        digitalWrite(pinINA, HIGH); digitalWrite(pinINB, LOW); analogWrite(pinEN, speed);
    } else if (speed < 0) {
        digitalWrite(pinINA, LOW); digitalWrite(pinINB, HIGH); analogWrite(pinEN, -speed);
    } else {
        digitalWrite(pinINA, LOW); digitalWrite(pinINB, LOW); analogWrite(pinEN, 0);
    }
}

void controlN20Motor(float ly, float lx) {
    float throttle = (abs(ly) > DEADZONE) ? ly : 0.0f;
    float steering = (abs(lx) > DEADZONE) ? lx : 0.0f;
    int leftPWM = (int)((throttle - steering) * MAX_PWM);
    int rightPWM = (int)((throttle + steering) * MAX_PWM);
    setMotor(leftPWM, N20_ENA, N20_IN1, N20_IN2);
    setMotor(rightPWM, N20_ENB, N20_IN3, N20_IN4);
    if (abs(throttle) > 0 || abs(steering) > 0) {
        Serial.printf("N20 L:%d R:%d\n", leftPWM, rightPWM);
    }
}

void controlServoMotor() {
    bool isPressedA = ble.getButtonA();
    bool isPressedY = ble.getButtonY();
    bool isPressedX = ble.getButtonX();
    bool isPressedB = ble.getButtonB();
    bool isPressedLB = ble.getLB();
    bool isPressedRB = ble.getRB();
    bool isPressedLT = ble.getLT();  // LT 立即停止

    // ★ 4個普通servo（長按轉動，放開反方向）
    if (isPressedA) {
        servoAngleA += servoDirectionA * servoSpeed;
        servoAngleA = constrain(servoAngleA, 0, 180);
        ServoA.write((int)servoAngleA);
        wasPressedA = true;
    } else if (wasPressedA) {
        servoDirectionA *= -1; wasPressedA = false;
        Serial.printf("A: %.1f\n", servoAngleA);
    }

    if (isPressedY) {
        servoAngleY += servoDirectionY * servoSpeed;
        servoAngleY = constrain(servoAngleY, 0, 180);
        ServoY.write((int)servoAngleY);
        wasPressedY = true;
    } else if (wasPressedY) {
        servoDirectionY *= -1; wasPressedY = false;
        Serial.printf("Y: %.1f\n", servoAngleY);
    }

    if (isPressedX) {
        servoAngleX += servoDirectionX * servoSpeed;
        servoAngleX = constrain(servoAngleX, 0, 180);
        ServoX.write((int)servoAngleX);
        wasPressedX = true;
    } else if (wasPressedX) {
        servoDirectionX *= -1; wasPressedX = false;
        Serial.printf("X: %.1f\n", servoAngleX);
    }

    if (isPressedB) {
        servoAngleB += servoDirectionB * servoSpeed;
        servoAngleB = constrain(servoAngleB, 0, 180);
        ServoB.write((int)servoAngleB);
        wasPressedB = true;
    } else if (wasPressedB) {
        servoDirectionB *= -1; wasPressedB = false;
        Serial.printf("B: %.1f\n", servoAngleB);
    }

    // ★ MG996R 新邏輯：LB按一下正轉、RB按一下反轉、LT立即停止

    // LB：按一下切換正轉/停止
    else if (isPressedLB && !wasLB) {  // 僅按下瞬間
        mgLB_Toggle = !mgLB_Toggle;
        if (mgLB_Toggle) {
            mgDirection = 1;  // 正轉
            Serial.println("MG996R LB: 正轉開始");
        } else {
            mgDirection = 0;  // 停止
            Serial.println("MG996R LB: 正轉停止");
        }
    }
    // RB：按一下切換反轉/停止
    else if (isPressedRB && !wasRB) {  // 僅按下瞬間
        mgRB_Toggle = !mgRB_Toggle;
        if (mgRB_Toggle) {
            mgDirection = -1;  // 反轉
            Serial.println("MG996R RB: 反轉開始");
        } else {
            mgDirection = 0;  // 停止
            Serial.println("MG996R RB: 反轉停止");
        }
    }

    // ★ 持續轉動（非阻塞）
    if (mgDirection != 0 && millis() - mgLastUpdate > MG_SPEED_DELAY) {
        mgAngle += mgDirection * servoSpeed;
        mgAngle = constrain(mgAngle, 0, 180);
        mg996r.writeAngle((int)mgAngle);
        mgLastUpdate = millis();
        Serial.printf("MG996R 轉動 dir=%d 角度: %.1f\n", mgDirection, mgAngle);
    }

    wasLB = isPressedLB;
    wasRB = isPressedRB;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    
    ble.begin();
    ServoA.setupServoMotor(); ServoY.setupServoMotor();
    ServoX.setupServoMotor(); ServoB.setupServoMotor();
    mg996r.setupMG996R();
    setupN20Motor();
    
    // Serial.println("=== 機器人控制啟動 ===");
    // Serial.println("- A/Y/X/B: 長按轉動伺服");
    // Serial.println("- LB: 按一下正轉MG996R / 再按停止");
    // Serial.println("- RB: 按一下反轉MG996R / 再按停止");
    // Serial.println("- LT: 立即停止MG996R");
    // Serial.println("- 左搖桿: N20馬達前後左右");
}

void loop() {
    ble.update();
    if (ble.isConnected()) {
        float lx = ble.getLeftX(), ly = ble.getLeftY();
        controlN20Motor(ly, lx);
        controlServoMotor();
    } else {
        Serial.print("."); 
        analogWrite(N20_ENA, 0); analogWrite(N20_ENB, 0);
        delay(500);
    }
    delay(50);
}
