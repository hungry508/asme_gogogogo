#ifndef MG996R_H
#define MG996R_H

#include <Arduino.h>
#include <ESP32Servo.h>

class MG996R {
private:
    Servo servo;
    int pin;
    float currentSpeed;
    int neutral; 

public:
    MG996R(int servoPin);
    void setupMG996R();
    void write(float speed);  // -1.0(全速反) ~ 0(停) ~ 1.0(全速正)
    void stop();
    float getSpeed();
};

#endif
