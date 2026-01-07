#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>
#include <N20motor.h>

// 全域物件
BleXboxController ble;
MotorControl motorControl(18);  // GPIO18 控制舵機
N20Motor n20;

// N20 馬達控制 (L298N)
#define N20_ENA 12   // PWM 速度
#define N20_IN1 14   // 方向1
#define N20_IN2 27   // 方向2
#define DEADZONE 0.15f

void setupN20Motor() {
    pinMode(N20_ENA, OUTPUT);
    pinMode(N20_IN1, OUTPUT);
    pinMode(N20_IN2, OUTPUT);
    analogWrite(N20_ENA, 0);  // 初始停止
}

void controlN20Motor(float leftY, float leftX) {
    // 死區處理
    float speed = (abs(leftY) > DEADZONE) ? leftY : 0.0f;
    
    int pwmSpeed = (int)(abs(speed) * 200);  // 最大 200 PWM 保護 N20
    
    if (pwmSpeed > 255) pwmSpeed = 255;
    
    if (speed > 0) {           // 前進
        analogWrite(N20_ENA, pwmSpeed);
        digitalWrite(N20_IN1, HIGH);
        digitalWrite(N20_IN2, LOW);
        Serial.printf("前進 %d PWM", pwmSpeed);
    } else if (speed < 0) {    // 後退
        analogWrite(N20_ENA, pwmSpeed);
        digitalWrite(N20_IN1, LOW);
        digitalWrite(N20_IN2, HIGH);
        Serial.printf("後退 %d PWM", pwmSpeed);
    } else {
        analogWrite(N20_ENA, 0);
        Serial.print("停止");
    }
    
    // X 軸狀態顯示
    if (abs(leftX) > DEADZONE) {
        Serial.printf(" 轉向:%.2f", leftX);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("左搖桿: N20 前後 | A/Y: 舵機轉動 | 按後蓋PAIR配對");
    
    ble.begin();           // BLE Xbox
    motorControl.setup();  // 舵機
    setupN20Motor();       // N20 馬達
    
    Serial.println("就緒！");
}

void loop() {
    ble.update();  // 必須更新狀態
    
    if (ble.isConnected()) {
        float lx = ble.getLeftX();
        float ly = ble.getLeftY();
        
        Serial.printf("[連線] LX:%.2f LY:%.2f RX:%.2f RY:%.2f LT:%.2f RT:%.2f | ",
                     lx, ly, ble.getRightX(), ble.getRightY(), ble.getLT(), ble.getRT());
        
        // ★★★ N20 馬達控制 ★★★
        controlN20Motor(ly, lx);
        
        // 原有舵機按鈕控制
        if (ble.getButtonA()) {
            Serial.print(" | A ");
            motorControl.turnMax();  // 舵機 180°
        }
        if (ble.getButtonB()) Serial.print(" | B ");
        if (ble.getButtonX()) Serial.print(" | X ");
        if (ble.getButtonY()) {
            Serial.print(" | Y ");
            motorControl.turnMin();  // 舵機 0°
        }
        if (ble.getLB()) Serial.print(" | LB ");
        if (ble.getRB()) Serial.print(" | RB ");
        if (ble.getDpadUp()) Serial.print(" | ↑ ");
        if (ble.getDpadDown()) Serial.print(" | ↓ ");
        if (ble.getDpadLeft()) Serial.print(" | ← ");
        if (ble.getDpadRight()) Serial.print(" | → ");
        
        Serial.println();
    } else {
        Serial.print(".");  // 掃描中
        analogWrite(N20_ENA, 0);  // 未連線停止 N20
        delay(500);
    }
    
    delay(50);  // 20Hz
}
