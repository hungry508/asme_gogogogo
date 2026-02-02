#ifndef MG996R_ANGLE_H
#define MG996R_ANGLE_H

#include <Arduino.h>
#include <ESP32Servo.h>

class MG996R_angle {
public:
    MG996R_angle(int servoPin);
    void setupMG996R();
    void writeAngle(int angle);  // 0~180角度控制+鎖死
    void stop();                 // 回90°鎖死
    int getAngle();              // 目前角度

private:
    int pin;
    Servo servo;
    int currentAngle;
};

#endif
