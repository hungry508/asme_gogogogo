#include "MG996R.h"

MG996R::MG996R(int servoPin) : pin(servoPin), currentSpeed(0.0f) {}

void MG996R::setupMG996R() {
    servo.setPeriodHertz(50);
    servo.attach(pin, 500, 2500);  // 360度脈寬範圍
    stop();  // 初始化停止
}

void MG996R::write(float speed) {
    currentSpeed = speed;
    if (speed > 0.0f) {
        int pwmValue = 90 + (int)(speed * 90);  // 90~180
        servo.write(pwmValue);
    } else if (speed < 0.0f) {
        int pwmValue = 90 + (int)(speed * 90);  // 0~90
        servo.write(pwmValue);
    } else {
        servo.write(90);  // 停止
    }
}

void MG996R::stop() {
    servo.write(90);
    currentSpeed = 0.0f;
}

float MG996R::getSpeed() {
    return currentSpeed;
}
