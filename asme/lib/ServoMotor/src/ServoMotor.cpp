#include "ServoMotor.h"

MotorControl::MotorControl(int pin) : _pin(pin) {}

void MotorControl::setup() {
    // 分配硬體定時器並初始化
    ESP32PWM::allocateTimer(0); 
    _servo.setPeriodHertz(50);
    _servo.attach(_pin, 500, 2400);
}

void MotorControl::turnMax() {
    _servo.write(180); // 轉到180度
}

void MotorControl::turnMin() {
    _servo.write(0); // 轉到0度
}