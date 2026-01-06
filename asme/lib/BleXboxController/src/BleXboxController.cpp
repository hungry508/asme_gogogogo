#include "../include/BleXboxController.h"

BleXboxController::BleXboxController() {
    controller = new XboxController();
}

void BleXboxController::begin() {      
    controller->begin();
}

void BleXboxController::update() {
    
    if (controller->isConnected()) {
        
        XboxControlsState s;
        controller->read(&s);         
        
        // 更新內部狀態
        leftX = s.leftStickX;
        leftY = s.leftStickY;
        rightX = s.rightStickX;
        rightY = s.rightStickY;
        LT = s.leftTrigger;
        RT = s.rightTrigger;

        buttonA = s.buttonA;
        buttonB = s.buttonB;
        buttonX = s.buttonX;
        buttonY = s.buttonY;
        LB = s.leftBumper;
        RB = s.rightBumper;
        L3 = s.leftStickButton;
        R3 = s.rightStickButton;
        dpadUp = s.dpadUp;
        dpadDown = s.dpadDown;
        dpadLeft = s.dpadLeft;
        dpadRight = s.dpadRight;
        share = s.shareButton;
        menu = s.menuButton;
        xbox = s.xboxButton;

        connected = true;
    } else {
        connected = false;
    }
}


bool BleXboxController::isConnected() { return connected; }
float BleXboxController::getLeftX() { return leftX; }
float BleXboxController::getLeftY() { return leftY; }
float BleXboxController::getRightX() { return rightX; }
float BleXboxController::getRightY() { return rightY; }
float BleXboxController::getLT() { return LT; }
float BleXboxController::getRT() { return RT; }
bool BleXboxController::getButtonA() { return buttonA; }
bool BleXboxController::getButtonB() { return buttonB; }
bool BleXboxController::getButtonX() { return buttonX; }
bool BleXboxController::getButtonY() { return buttonY; }
bool BleXboxController::getLB() { return LB; }
bool BleXboxController::getRB() { return RB; }
bool BleXboxController::getL3() { return L3; }
bool BleXboxController::getR3() { return R3; }
bool BleXboxController::getDpadUp() { return dpadUp; }
bool BleXboxController::getDpadDown() { return dpadDown; }
bool BleXboxController::getDpadLeft() { return dpadLeft; }
bool BleXboxController::getDpadRight() { return dpadRight; }
bool BleXboxController::getShare() { return share; }
bool BleXboxController::getMenu() { return menu; }
bool BleXboxController::getXbox() { return xbox; }
