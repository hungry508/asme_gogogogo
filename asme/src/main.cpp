#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>
#include <N20motor.h>
#include <MG996R_angle.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================= 硬體物件 =================
BleXboxController ble;
MotorControl ServoA(18);
MotorControl ServoY(19);
MotorControl ServoX(21);
MotorControl ServoB(22);
MG996R_angle mg996r(23);

// N20 腳位定義
#define N20_ENA 12
#define N20_IN1 14
#define N20_IN2 27
#define N20_ENB 33
#define N20_IN3 26
#define N20_IN4 25
#define MAX_PWM 200

// ================= 全域變數 =================
volatile float g_joyLX = 0;
volatile float g_joyLY = 0;

volatile int g_dirA = 0;
volatile int g_dirY = 0;
volatile int g_dirX = 0;
volatile int g_dirB = 0;

// MG996R 方向變數 (1=正轉, -1=反轉, 0=不動)
volatile int g_mgDirection = 0; 

// 舵機當前角度
float currentAngleA = 90;
float currentAngleY = 90;
float currentAngleX = 90;
float currentAngleB = 90;
float currentAngleMG = 90; // MG996R 的當前角度

// ★ 速度設定 (數字越大，手臂移動越快)
const float SERVO_SPEED_STEP = 2.0; 

// ================= 輔助函式 =================
void setupN20Motor() {
    pinMode(N20_ENA, OUTPUT); pinMode(N20_IN1, OUTPUT); pinMode(N20_IN2, OUTPUT);
    pinMode(N20_ENB, OUTPUT); pinMode(N20_IN3, OUTPUT); pinMode(N20_IN4, OUTPUT);
}

void setMotor(int speed, int pinEN, int pinINA, int pinINB) {
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;
    if (speed > 0) {
        digitalWrite(pinINA, HIGH); digitalWrite(pinINB, LOW); analogWrite(pinEN, speed);
    } else if (speed < 0) {
        digitalWrite(pinINA, LOW); digitalWrite(pinINB, HIGH); analogWrite(pinEN, -speed);
    } else {
        digitalWrite(pinINA, LOW); digitalWrite(pinINB, LOW); analogWrite(pinEN, 0);
    }
}

// ================= 任務 1：輸入處理 (核心 0) =================
void Task_Input(void *pvParameters) {
    bool wasPressedA=false, wasPressedY=false, wasPressedX=false, wasPressedB=false;
    int toggleDirA=1, toggleDirY=1, toggleDirX=1, toggleDirB=1;

    for (;;) {
        ble.update(); 
        
        if (ble.isConnected()) {
            g_joyLX = ble.getLeftX();
            g_joyLY = ble.getLeftY();

            // A/B/X/Y (維持原本邏輯)
            if (ble.getButtonA()) { g_dirA = toggleDirA; wasPressedA = true; } 
            else { g_dirA = 0; if(wasPressedA){ toggleDirA *= -1; wasPressedA = false; }}

            if (ble.getButtonY()) { g_dirY = toggleDirY; wasPressedY = true; } 
            else { g_dirY = 0; if(wasPressedY){ toggleDirY *= -1; wasPressedY = false; }}

            if (ble.getButtonX()) { g_dirX = toggleDirX; wasPressedX = true; } 
            else { g_dirX = 0; if(wasPressedX){ toggleDirX *= -1; wasPressedX = false; }}

            if (ble.getButtonB()) { g_dirB = toggleDirB; wasPressedB = true; } 
            else { g_dirB = 0; if(wasPressedB){ toggleDirB *= -1; wasPressedB = false; }}

            // ★★★ MG996R 控制邏輯 (已修改為 LB / RB) ★★★
            // LB 和 RB 是 bool (true/false)，不需要 threshold
            bool isLB = ble.getLB(); 
            bool isRB = ble.getRB(); 

            if (isLB) {
                g_mgDirection = 1;  // 按住 LB -> 正轉
            } 
            else if (isRB) {
                g_mgDirection = -1; // 按住 RB -> 反轉
            } 
            else {
                g_mgDirection = 0;  // 都沒按 -> 保持不動
            }

        } else {
            // 斷線保護
            g_mgDirection = 0;
            g_dirA=0; g_dirY=0; g_dirX=0; g_dirB=0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= 任務 2：馬達控制 (核心 1) =================
void Task_Motors(void *pvParameters) {
    for (;;) {
        // N20 控制
        float rawY = (abs(g_joyLY) > 0.15f) ? g_joyLY : 0;
        float rawX = (abs(g_joyLX) > 0.15f) ? g_joyLX : 0;
        int leftPWM = (int)((rawY - rawX) * MAX_PWM);
        int rightPWM = (int)((rawY + rawX) * MAX_PWM);
        setMotor(leftPWM, N20_ENA, N20_IN1, N20_IN2);
        setMotor(rightPWM, N20_ENB, N20_IN3, N20_IN4);

        // A/B/X/Y 舵機
        if (g_dirA != 0) { currentAngleA += g_dirA * SERVO_SPEED_STEP; currentAngleA = constrain(currentAngleA, 0, 180); ServoA.write((int)currentAngleA); }
        if (g_dirY != 0) { currentAngleY += g_dirY * SERVO_SPEED_STEP; currentAngleY = constrain(currentAngleY, 0, 180); ServoY.write((int)currentAngleY); }
        if (g_dirX != 0) { currentAngleX += g_dirX * SERVO_SPEED_STEP; currentAngleX = constrain(currentAngleX, 0, 180); ServoX.write((int)currentAngleX); }
        if (g_dirB != 0) { currentAngleB += g_dirB * SERVO_SPEED_STEP; currentAngleB = constrain(currentAngleB, 0, 180); ServoB.write((int)currentAngleB); }

        // ★★★ MG996R 動作更新 (180度機械手臂專用) ★★★
        if (g_mgDirection != 0) {
            // 按住 LB/RB 時更新角度
            currentAngleMG += g_mgDirection * SERVO_SPEED_STEP;
            currentAngleMG = constrain(currentAngleMG, 0, 180);
            mg996r.writeAngle((int)currentAngleMG);
        }
        // 放開按鈕時：什麼都不做，馬達會維持在 currentAngleMG 的位置

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ================= 主程式 =================
void setup() {
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    
    ServoA.setupServoMotor(); ServoY.setupServoMotor();
    ServoX.setupServoMotor(); ServoB.setupServoMotor();
    mg996r.setupMG996R();
    setupN20Motor();
    
    ble.begin();
    Serial.println("系統啟動 (180度模式) - LB/RB 控制手臂升降"); // 修改 Serial 提示

    xTaskCreatePinnedToCore(Task_Input, "InputTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Task_Motors, "MotorTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}