#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>

// 全域物件
BleXboxController ble;
MotorControl motorControl(18);  // GPIO18 控制馬達

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("按 Xbox 手把後蓋 PAIR 鈕 3 秒配對...");
    ble.begin();  // 開始 BLE 掃描
    motorControl.setup();  // 設定馬達腳位
}

void loop() {
    ble.update();  // 更新狀態（必填！）
    if (ble.isConnected()) {
        // 搖桿值
        Serial.printf("[連線] LX:%.2f LY:%.2f RX:%.2f RY:%.2f LT:%.2f RT:%.2f | ",
                     ble.getLeftX(), ble.getLeftY(), 
                     ble.getRightX(), ble.getRightY(),
                      ble.getLT(), ble.getRT());

        // 按鈕狀態
        if (ble.getButtonA()){
            Serial.print("A ");
            motorControl.turnMax();//ServoMotor 轉到180度
        }

        if (ble.getButtonB()) Serial.print("B ");
        if (ble.getButtonX()) Serial.print("X ");

        if (ble.getButtonY()){
            Serial.print("Y ");
            motorControl.turnMin(); // ServoMotor 轉到0度
        }

        if (ble.getLB()) Serial.print("LB ");
        if (ble.getRB()) Serial.print("RB ");
        if (ble.getDpadUp()) Serial.print("↑ ");
        if (ble.getDpadDown()) Serial.print("↓ ");
        if (ble.getDpadLeft()) Serial.print("← ");
        if (ble.getDpadRight()) Serial.print("→ ");
        
        Serial.println();
    } else {
        Serial.print(".");  // 掃描中印點點
        delay(500);
    }
    
    delay(50);  // 20Hz 更新
}
