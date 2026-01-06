#ifndef BLE_XBOX_CONTROLLER_H
#define BLE_XBOX_CONTROLLER_H

#include "Arduino.h"
#include <BLEGamepadClient.h>

class BleXboxController {
public:
    BleXboxController();
    void begin();
    void update();  // 在 loop() 呼叫

    bool isConnected();

    // 搖桿 (-1.0 ~ 1.0)
    float getLeftX(); float getLeftY();
    float getRightX(); float getRightY();
    float getLT(); float getRT();

    // 全按鈕
    bool getButtonA(); bool getButtonB(); bool getButtonX(); bool getButtonY();
    bool getLB(); bool getRB();
    bool getL3(); bool getR3();  // 搖桿按
    bool getDpadUp(); bool getDpadDown(); bool getDpadLeft(); bool getDpadRight();
    bool getShare(); bool getMenu(); bool getXbox();

private:

    XboxController* controller;     
    bool connected = false;

    String targetMAC = "0c:35:26:be:05:1b";
    // 內部狀態
    
    float leftX, leftY, rightX, rightY, LT, RT;
    bool buttonA, buttonB, buttonX, buttonY;
    bool LB, RB, L3, R3;
    bool dpadUp, dpadDown, dpadLeft, dpadRight;
    bool share, menu, xbox;
};

#endif
