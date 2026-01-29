#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>
#include <MG996R.h>
#include <N20motor.h>
#include <ESP32Servo.h>

BleXboxController ble;
MotorControl ServoA(18);
MotorControl ServoY(19);
MotorControl ServoX(21);
MotorControl ServoB(22);
N20Motor n20;
MG996R mg996r(23);

// ★★★ MG996R控制變數（修復編譯錯誤） ★★★
static float mgTargetSpeed = 0.0f;      // 0停, 1.0正轉, -1.0反轉
static bool lastLBState = false;        // 上次LB狀態
static unsigned long lastLBPress = 0;   // 上次按下時間
const unsigned long DEBOUNCE = 300;     // 防抖300ms


int servoDirectionA = 1;
int servoDirectionY = 1;
int servoDirectionX = 1;
int servoDirectionB = 1;
bool wasPressedA = false;
bool wasPressedY = false;
bool wasPressedX = false;
bool wasPressedB = false;
const float servoSpeed = 4;
float servoAngleA = 0;
float servoAngleY = 0;
float servoAngleX = 0;
float servoAngleB = 0;

static int mgDirection = 1;
static bool wasPressedLB = false;
static float mgAngle = 90.0f;

#define N20_ENA 12
#define N20_IN1 14
#define N20_IN2 27
#define N20_ENB 33
#define N20_IN3 26
#define N20_IN4 25
#define DEADZONE 0.15f
#define MAX_PWM 200

void setupN20Motor() {
    pinMode(N20_ENA, OUTPUT);
    pinMode(N20_IN1, OUTPUT);
    pinMode(N20_IN2, OUTPUT);
    pinMode(N20_ENB, OUTPUT);
    pinMode(N20_IN3, OUTPUT);
    pinMode(N20_IN4, OUTPUT);
    analogWrite(N20_ENA, 0);
    analogWrite(N20_ENB, 0);
}

void setMotor(int speed, int pinEN, int pinINA, int pinINB) {
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;

    if (speed > 0) {
        digitalWrite(pinINA, HIGH);
        digitalWrite(pinINB, LOW);
        analogWrite(pinEN, speed);
    } else if (speed < 0) {
        digitalWrite(pinINA, LOW);
        digitalWrite(pinINB, HIGH);
        analogWrite(pinEN, -speed);
    } else {
        digitalWrite(pinINA, LOW);
        digitalWrite(pinINB, LOW);
        analogWrite(pinEN, 0);
    }
}

void controlN20Motor(float ly, float lx) {
    float throttle = (abs(ly) > DEADZONE) ? ly : 0.0f;
    float steering = (abs(lx) > DEADZONE) ? lx : 0.0f;

    float leftSpeedVal = throttle - steering;
    float rightSpeedVal = throttle + steering;

    int leftPWM = (int)(leftSpeedVal * MAX_PWM);
    int rightPWM = (int)(rightSpeedVal * MAX_PWM);

    setMotor(leftPWM, N20_ENA, N20_IN1, N20_IN2);
    setMotor(rightPWM, N20_ENB, N20_IN3, N20_IN4);

    if (abs(throttle) > 0 || abs(steering) > 0) {
        Serial.printf("L: %d | R: %d (Y:%.2f X:%.2f)\n", leftPWM, rightPWM, throttle, steering);
    }
}

void controlServoMotor() {
    bool isPressedA = ble.getButtonA();
    bool isPressedY = ble.getButtonY();
    bool isPressedX = ble.getButtonX();
    bool isPressedB = ble.getButtonB();
    bool isPressedLB = ble.getLB();
    // A
    if (isPressedA) {
        servoAngleA += (servoDirectionA * servoSpeed);
        if (servoAngleA >= 180) {
            servoAngleA = 180;
        } else if (servoAngleA <= 0) {
            servoAngleA = 0;
        }
        ServoA.write((int)servoAngleA);
        wasPressedA = true;
    } else {
        if (wasPressedA) {
            servoDirectionA *= -1;
            wasPressedA = false;
            Serial.printf("A 放開，下次方向反轉，目前角度: %.1f\n", servoAngleA);
        }
    }

    // Y
    if (isPressedY) {
        servoAngleY += (servoDirectionY * servoSpeed);
        if (servoAngleY >= 180) {
            servoAngleY = 180;
        } else if (servoAngleY <= 0) {
            servoAngleY = 0;
        }
        ServoY.write((int)servoAngleY);
        wasPressedY = true;
    } else {
        if (wasPressedY) {
            servoDirectionY *= -1;
            wasPressedY = false;
            Serial.printf("Y 放開，下次方向反轉，目前角度: %.1f\n", servoAngleY);
        }
    }

    // X
    if (isPressedX) {
        servoAngleX += (servoDirectionX * servoSpeed);
        if (servoAngleX >= 180) {
            servoAngleX = 180;
        } else if (servoAngleX <= 0) {
            servoAngleX = 0;
        }
        ServoX.write((int)servoAngleX);
        wasPressedX = true;
    } else {
        if (wasPressedX) {
            servoDirectionX *= -1;
            wasPressedX = false;
            Serial.printf("X 放開，下次方向反轉，目前角度: %.1f\n", servoAngleX);
        }
    }

    // B
    if (isPressedB) {
        servoAngleB += (servoDirectionB * servoSpeed);
        if (servoAngleB >= 180) {
            servoAngleB = 180;
        } else if (servoAngleB <= 0) {
            servoAngleB = 0;
        }
        ServoB.write((int)servoAngleB);
        wasPressedB = true;
    } else {
        if (wasPressedB) {
            servoDirectionB *= -1;
            wasPressedB = false;
            Serial.printf("B 放開，下次方向反轉，目前角度: %.1f\n", servoAngleB);
        }
    }

    // MG996R：LB 控制
    // mgTargetSpeed=0（正轉）變 狀態變1
    // mgTargetSpeed=1（反轉）變 狀態變2
    // mgTargetSpeed=-1（停止）變 狀態變0
    if (isPressedLB && !lastLBState && (millis() - lastLBPress > DEBOUNCE)) {
        if (mgTargetSpeed == 0.0f) {
            mgTargetSpeed = 1.0f;  // 正轉
            Serial.println("MG996R: 正轉啟動!");
        } else if (mgTargetSpeed == 1.0f) {
            mgTargetSpeed = -1.0f; // 反轉
            Serial.println("MG996R: 反轉啟動!");
        } else {
            mgTargetSpeed = 0.0f;  // 停
            Serial.println("MG996R: 停止!");
        }
        lastLBPress = millis();
    }

    lastLBState = isPressedLB;// 記錄當前狀態

    // 按住時持續目標速度，放開立即停止
    if (isPressedLB) {
        mg996r.write(mgTargetSpeed);  // 持續輸出速度
        Serial.printf("MG996R 持續速度: %.1f\n", mgTargetSpeed);
    } else {
        mg996r.stop();  // 放開立即停止
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    ble.begin();
    ServoA.setupServoMotor();
    ServoY.setupServoMotor();
    ServoX.setupServoMotor();
    ServoB.setupServoMotor();
    mg996r.setupMG996R();
    setupN20Motor();
}

void loop() {
    ble.update();

    if (ble.isConnected()) {
        float lx = ble.getLeftX();
        float ly = ble.getLeftY();

        controlN20Motor(ly, lx);
        controlServoMotor();

        Serial.println();
    } else {
        Serial.print(".");
        analogWrite(N20_ENA, 0);
        delay(500);
    }

    delay(50);
}
