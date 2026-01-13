#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ESP32Servo.h>

class MotorControl {
public:
    MotorControl(int pin);
    void setupServoMotor();
    void write(int angle);
private:
    Servo _servo;
    int _pin;
};

#endif