#include "ServoMotor.h"

MotorControl::MotorControl(int pin) : _pin(pin) {}

void MotorControl::setup() {
    // 設定舵機參數
    _servo.setPeriodHertz(50);
    _servo.attach(_pin, 500, 2400);
    ESP32PWM::allocateTimer(0);
    // 初始位置
    _servo.write(0);
    delay(50);
}

void MotorControl::write(int angle) {
    if (angle >= 0 && angle <= 180) {
        _servo.write(angle);
    }
}
