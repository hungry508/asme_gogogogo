#include "N20motor.h"

N20Motor n20;  // 定義全域 N20Motor 實例

void N20Motor::init() {
    // 初始化 N20 馬達引腳
    pinMode(N20_ENA, OUTPUT);
    pinMode(N20_IN1, OUTPUT);
    pinMode(N20_IN2, OUTPUT);
    
    // 初始化 Xbox 控制器
    xbox.begin();
    
    Serial.begin(115200);
    // Serial.println("N20 馬達 + Xbox 控制就緒，等待手把連線...");
}

void N20Motor::update() {
    xbox.update();  // 更新控制器狀態
    
    if (xbox.isConnected()) {
        // 讀取左蘑菇頭 (主要 Y 軸前後，X 軸轉向)
        float leftY = xbox.getLeftY();  // -1.0(後退) ~ 1.0(前進)
        float leftX = xbox.getLeftX();  // 轉向調整
        
        // 死區處理
        float speed = (abs(leftY) > DEADZONE) ? leftY : 0.0f;
        
        int pwmSpeed = (int)(abs(speed) * 255.0f);
        if (pwmSpeed > 255) pwmSpeed = 255;
        
        // 馬達控制邏輯
        if (speed > 0) {           // 前進
            analogWrite(N20_ENA, pwmSpeed);
            digitalWrite(N20_IN1, HIGH);
            digitalWrite(N20_IN2, LOW);
        } else if (speed < 0) {    // 後退
            analogWrite(N20_ENA, pwmSpeed);
            digitalWrite(N20_IN1, LOW);
            digitalWrite(N20_IN2, HIGH);
        } else {                   // 停止
            analogWrite(N20_ENA, 0);
        }
        
    } else {
        analogWrite(N20_ENA, 0);  // 未連線停止
        Serial.println("Xbox 未連線，N20 停止");
    }
}
