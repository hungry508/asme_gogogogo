#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ESP32Servo.h>

class MotorControl {
public:
    MotorControl(int pin);
    void setup();
    void turnMax();//轉到180度
    void turnMin();//轉到0度
private:
    Servo _servo;
    int _pin;
};

#endif