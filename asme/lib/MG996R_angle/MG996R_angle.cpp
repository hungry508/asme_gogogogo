#include "MG996R_angle.h"

MG996R_angle::MG996R_angle(int servoPin) : pin(servoPin), currentAngle(90) {}

void MG996R_angle::setupMG996R() {
    servo.setPeriodHertz(50);
    servo.attach(pin, 500, 2500);  // 180°範圍
    servo.write(90);  // 開機中立
}

void MG996R_angle::writeAngle(int angle) {
    currentAngle = constrain(angle, 0, 180);
    servo.write(currentAngle);  // 精準定位+強力鎖死
}

void MG996R_angle::stop() {
    servo.write(90);
    currentAngle = 90;
}

int MG996R_angle::getAngle() {
    return currentAngle;
}
