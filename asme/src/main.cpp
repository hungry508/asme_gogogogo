#include <Arduino.h>
#include <BleXboxController.h> 
#include <N20motor.h>
#include <MG996R_angle.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================= 硬體物件 =================
BleXboxController ble;

// 第一顆 MG996R (接 Pin 23)
MG996R_angle mg996r_1(23);

// 第二顆 MG996R (接 Pin 22)
MG996R_angle mg996r_2(22);

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

// MG996R 方向變數 (1=正轉, -1=反轉, 0=不動)
volatile int g_mg1Direction = 0; // 第一顆 (LB/RB)
volatile int g_mg2Direction = 0; // 第二顆 (Y/A)

// 舵機當前角度
float currentAngleMG1 = 90; // 第一顆角度
float currentAngleMG2 = 90; // 第二顆角度

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
    for (;;) {
        ble.update(); 
        
        if (ble.isConnected()) {
            g_joyLX = ble.getLeftX();
            g_joyLY = ble.getLeftY();

            // --- 第一顆 MG996R 控制 (LB / RB) ---
            if (ble.getLB()) g_mg1Direction = 1;
            else if (ble.getRB()) g_mg1Direction = -1;
            else g_mg1Direction = 0;

            // --- 第二顆 MG996R 控制 (Y / A) ---
            if (ble.getButtonY()) g_mg2Direction = 1;
            else if (ble.getButtonA()) g_mg2Direction = -1;
            else g_mg2Direction = 0;

        } else {
            // 斷線保護
            g_mg1Direction = 0;
            g_mg2Direction = 0;
            g_joyLX = 0;
            g_joyLY = 0;
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

        // --- 第一顆 MG996R 動作更新 ---
        if (g_mg1Direction != 0) {
            currentAngleMG1 += g_mg1Direction * SERVO_SPEED_STEP;
            currentAngleMG1 = constrain(currentAngleMG1, 0, 180);
            mg996r_1.writeAngle((int)currentAngleMG1);
        }

        // --- 第二顆 MG996R 動作更新 ---
        if (g_mg2Direction != 0) {
            currentAngleMG2 += g_mg2Direction * SERVO_SPEED_STEP;
            currentAngleMG2 = constrain(currentAngleMG2, 0, 180);
            mg996r_2.writeAngle((int)currentAngleMG2);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ================= 主程式 =================
void setup() {
    Serial.begin(115200);
    
    // 分配 ESP32 定時器給伺服馬達使用
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    
    // 初始化 MG996R
    mg996r_1.setupMG996R(); 
    mg996r_2.setupMG996R(); 

    // 初始化 N20
    setupN20Motor();
    
    ble.begin();
    Serial.println("系統啟動 (僅 MG996R 模式)");
    Serial.println("- LB/RB 控制第1顆 (Pin 23)");
    Serial.println("- Y/A   控制第2顆 (Pin 22)");

    xTaskCreatePinnedToCore(Task_Input, "InputTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Task_Motors, "MotorTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // 核心 0 與 1 已在處理任務，主迴圈僅需休眠
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}