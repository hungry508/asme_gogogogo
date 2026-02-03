#include "MG996R_angle.h"

MG996R_angle::MG996R_angle(int servoPin) : pin(servoPin), currentAngle(90) {}

void MG996R_angle::setupMG996R() {
    servo.setPeriodHertz(50);
    // 建議改成 2400 比較保險，避免極限角度卡住發熱
    servo.attach(pin, 500, 2400);  
    
    // 開機時的預設位置
    // 如果你的手臂開機時應該是垂下的(0度)，這裡要改成 write(0)
    servo.write(90);  
}

void MG996R_angle::writeAngle(int angle) {
    // 雙重保險，限制角度
    currentAngle = constrain(angle, 0, 180);
    servo.write(currentAngle); 
}

// 取得目前角度 (這在 FreeRTOS 邏輯中很有用)
int MG996R_angle::getAngle() {
    return currentAngle;
}

// ★★★ 移除了 stop() 函式，避免誤觸導致手臂甩回中間 ★★★