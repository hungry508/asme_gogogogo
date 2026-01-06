#include <Arduino.h>
#include <BleXboxController.h>  // 你的 library

BleXboxController ble;  // 全域物件

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("按 Xbox 手把後蓋 PAIR 鈕 3 秒配對...");
    ble.begin();  // 開始 BLE 掃描
}

void loop() {
    ble.update();  // 更新狀態（必填！）
    if (ble.isConnected()) {
        // 搖桿值
        Serial.printf("[連線] LX:%.2f LY:%.2f RX:%.2f RT:%.2f | ",
                     ble.getLeftX(), ble.getLeftY(), 
                     ble.getRightX(), ble.getRT());
        
        // 按鈕狀態
        if (ble.getButtonA()) Serial.print("A ");
        if (ble.getButtonB()) Serial.print("B ");
        if (ble.getButtonX()) Serial.print("X ");
        if (ble.getButtonY()) Serial.print("Y ");
        if (ble.getLB()) Serial.print("LB ");
        if (ble.getRB()) Serial.print("RB ");
        if (ble.getDpadUp()) Serial.print("↑ ");
        
        Serial.println();
    } else {
        Serial.print(".");  // 掃描中印點點
        delay(500);
    }
    
    delay(50);  // 20Hz 更新
}
