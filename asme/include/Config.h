#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ================= N20 馬達腳位定義 (L298N) =================
#define N20_ENA 12
#define N20_IN1 14
#define N20_IN2 27
#define N20_ENB 33
#define N20_IN3 26
#define N20_IN4 25
#define MAX_PWM 200  // N20 最大輸出限速 (0-255)

// ================= MG996R 舵機腳位定義 =================
#define MG180_1_PIN 23  // 180度馬達 1 (LB/RB)
#define MG180_2_PIN 22  // 180度馬達 2 (Y/A)
#define MG360_1_PIN 21  // 360度馬達 1 (X/B)
#define MG360_2_PIN 19  // 360度馬達 2 (Dpad Up/Down)

// ================= 控制參數設定 =================
const float SERVO_SPEED_STEP = 2.0;    // 180度舵機每步旋轉角度
const float MG360_RUN_SPEED = 0.5f;    // 360度舵機運轉速度 (0.0 ~ 1.0)
const float JOYSTICK_DEADZONE = 0.15f; // 搖桿死區防抖

#endif