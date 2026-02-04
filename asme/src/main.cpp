#include <Arduino.h>
#include <BleXboxController.h> 
#include <N20motor.h>
#include <MG996R_angle.h>  
#include "MG996R.h"        
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Config.h"

// ================= 硬體物件 =================
BleXboxController ble;

MG996R_angle mg180_1(MG180_1_PIN);
MG996R_angle mg180_2(MG180_2_PIN);
MG996R mg360_1(MG360_1_PIN);
MG996R mg360_2(MG360_2_PIN);

// ================= 全域變數 =================
volatile float g_joyLX = 0;
volatile float g_joyLY = 0;
volatile int g_mg180_1_Dir = 0; 
volatile int g_mg180_2_Dir = 0; 
volatile float g_mg360_1_Speed = 0.0f; 
volatile float g_mg360_2_Speed = 0.0f; 

float currentAngle180_1 = 90;
float currentAngle180_2 = 90;

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

            // 180度控制
            if (ble.getLB()) g_mg180_1_Dir = 1;
            else if (ble.getRB()) g_mg180_1_Dir = -1;
            else g_mg180_1_Dir = 0;

            if (ble.getButtonY()) g_mg180_2_Dir = 1;
            else if (ble.getButtonA()) g_mg180_2_Dir = -1;
            else g_mg180_2_Dir = 0;

            // 360度 #1 (X/B)
            if (ble.getButtonX()) g_mg360_1_Speed = MG360_RUN_SPEED; 
            else if (ble.getButtonB()) g_mg360_1_Speed = -MG360_RUN_SPEED; 
            else g_mg360_1_Speed = 0.0f; 

            // 360度 #2 (Dpad)
            if (ble.getDpadUp()) g_mg360_2_Speed = MG360_RUN_SPEED;
            else if (ble.getDpadDown()) g_mg360_2_Speed = -MG360_RUN_SPEED;
            else g_mg360_2_Speed = 0.0f;

        } else {
            g_mg180_1_Dir = 0; g_mg180_2_Dir = 0;
            g_mg360_1_Speed = 0; g_mg360_2_Speed = 0;
            g_joyLX = 0; g_joyLY = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= 任務 2：馬達控制 (核心 1) =================
void Task_Motors(void *pvParameters) {
    for (;;) {
        // N20 車底盤
        float rawY = (abs(g_joyLY) > JOYSTICK_DEADZONE) ? g_joyLY : 0;
        float rawX = (abs(g_joyLX) > JOYSTICK_DEADZONE) ? g_joyLX : 0;
        int leftPWM = (int)((rawY - rawX) * MAX_PWM);
        int rightPWM = (int)((rawY + rawX) * MAX_PWM);
        setMotor(leftPWM, N20_ENA, N20_IN1, N20_IN2);
        setMotor(rightPWM, N20_ENB, N20_IN3, N20_IN4);

        // 180度更新
        if (g_mg180_1_Dir != 0) {
            currentAngle180_1 = constrain(currentAngle180_1 + g_mg180_1_Dir * SERVO_SPEED_STEP, 0, 180);
            mg180_1.writeAngle((int)currentAngle180_1);
        }
        if (g_mg180_2_Dir != 0) {
            currentAngle180_2 = constrain(currentAngle180_2 + g_mg180_2_Dir * SERVO_SPEED_STEP, 0, 180);
            mg180_2.writeAngle((int)currentAngle180_2);
        }

        // 360度更新
        mg360_1.write(g_mg360_1_Speed);
        mg360_2.write(g_mg360_2_Speed);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ================= 主程式 =================
void setup() {
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    
    mg180_1.setupMG996R(); 
    mg180_2.setupMG996R(); 
    mg360_1.setupMG996R();
    mg360_2.setupMG996R();

    setupN20Motor();
    ble.begin();
    
    xTaskCreatePinnedToCore(Task_Input, "InputTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Task_Motors, "MotorTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}