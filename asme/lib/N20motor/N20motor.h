#ifndef N20MOTOR_H
#define N20MOTOR_H

#include "Arduino.h"
#include "BleXboxController.h"  // 使用您的 header

// N20 馬達引腳定義 (L298N Channel A)
#define N20_ENA 12   // PWM 速度控制
#define N20_IN1 14   // 方向 1
#define N20_IN2 27   // 方向 2

// 搖桿死區範圍
#define DEADZONE 0.15f

class N20Motor {
private:
    BleXboxController xbox;  // 您的 Xbox 控制器
    bool initialized = false;

public:
    void init();     // 初始化
    void update();   // 在 loop() 呼叫，讀搖桿控制馬達
};

extern N20Motor n20;  // 全域實例

#endif
