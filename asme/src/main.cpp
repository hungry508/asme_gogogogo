#include <Arduino.h>
#include <BleXboxController.h> 
#include <ServoMotor.h>
#include <N20motor.h>
#include <ESP32Servo.h>

// 全域物件
BleXboxController ble;
MotorControl ServoA(18);//GPIO18
MotorControl ServoY(19);//GPIO19
MotorControl ServoX(25);//GPIO25
MotorControl ServoB(26);//GPIO26
N20Motor n20;

//Servo
int servoDirectionA = 1;
int servoDirectionY = 1;
int servoDirectionX = 1;
int servoDirectionB = 1;
bool wasPressedA = false;
bool wasPressedY = false;
bool wasPressedX = false;
bool wasPressedB = false;
const float servoSpeed = 4;//速度
float servoAngleA = 0;
float servoAngleY = 0;
float servoAngleX = 0;
float servoAngleB = 0;

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

void controlServoMotor() {
    bool isPressedA = ble.getButtonA(); // 使用 A 鍵控制
    bool isPressedY = ble.getButtonY(); // 使用 Y 鍵控制
    bool isPressedX = ble.getButtonX(); // 使用 X 鍵控制
    bool isPressedB = ble.getButtonB(); // 使用 B 鍵控制

        //A
        if (isPressedA) {
            // --- 動作中 ---
            // 根據方向緩慢增減角度
            servoAngleA += (servoDirectionA * servoSpeed);
            
            // 邊界保護：防止超出 0~180 度
            if (servoAngleA >= 180) {
                servoAngleA = 180;
                // 到達頂點時，如果還按著，可以選擇停住或自動反轉
            } else if (servoAngleA <= 0) {
                servoAngleA = 0;
            }

            ServoA.write((int)servoAngleA);
            wasPressedA = true; // 標記目前正在按住
        } 
        else {
            // --- 放開按鈕的一瞬間 ---
            if (wasPressedA) {
                // 只有在剛放開的那一刻，切換下一次的方向
                servoDirectionA *= -1; 
                wasPressedA = false; 
                Serial.printf("放開按鈕，下次方向將反轉。目前角度: %.1f\n", servoAngleA);
            }
        }

        //Y
        if (isPressedY) {
            // --- 動作中 ---
            // 根據方向緩慢增減角度
            servoAngleY += (servoDirectionY * servoSpeed);
            
            // 邊界保護：防止超出 0~180 度
            if (servoAngleY >= 180) {
                servoAngleY = 180;
                // 到達頂點時，如果還按著，可以選擇停住或自動反轉
            } else if (servoAngleY <= 0) {
                servoAngleY = 0;
            }

            ServoY.write((int)servoAngleY);
            wasPressedY = true; // 標記目前正在按住
        } 
        else {
            // --- 放開按鈕的一瞬間 ---
            if (wasPressedY) {
                // 只有在剛放開的那一刻，切換下一次的方向
                servoDirectionY *= -1; 
                wasPressedY = false; 
                Serial.printf("放開按鈕，下次方向將反轉。目前角度: %.1f\n", servoAngleY);
            }
        }

        //X
        if (isPressedX) {
            // --- 動作中 ---
            // 根據方向緩慢增減角度
            servoAngleX += (servoDirectionX * servoSpeed);
            
            // 邊界保護：防止超出 0~180 度
            if (servoAngleX >= 180) {
                servoAngleX = 180;
                // 到達頂點時，如果還按著，可以選擇停住或自動反轉
            } else if (servoAngleX <= 0) {
                servoAngleX = 0;
            }

            ServoX.write((int)servoAngleX);
            wasPressedX = true; // 標記目前正在按住
        } 
        else {
            // --- 放開按鈕的一瞬間 ---
            if (wasPressedX) {
                // 只有在剛放開的那一刻，切換下一次的方向
                servoDirectionX *= -1; 
                wasPressedX = false; 
                Serial.printf("放開按鈕，下次方向將反轉。目前角度: %.1f\n", servoAngleX);
            }
        }

        //B
        if (isPressedB) {
            // --- 動作中 ---
            // 根據方向緩慢增減角度
            servoAngleB += (servoDirectionB * servoSpeed);
            
            // 邊界保護：防止超出 0~180 度
            if (servoAngleB >= 180) {
                servoAngleB = 180;
                // 到達頂點時，如果還按著，可以選擇停住或自動反轉
            } else if (servoAngleB <= 0) {
                servoAngleB = 0;
            }

            ServoB.write((int)servoAngleB);
            wasPressedB = true; // 標記目前正在按住
        } 
        else {
            // --- 放開按鈕的一瞬間 ---
            if (wasPressedB) {
                // 只有在剛放開的那一刻，切換下一次的方向
                servoDirectionB *= -1; 
                wasPressedB = false; 
                Serial.printf("放開按鈕，下次方向將反轉。目前角度: %.1f\n", servoAngleB);
            }
        }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // 在舵機初始化前統一分配所有定時器
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    ble.begin();           // BLE Xbox
    ServoA.setupServoMotor();  // 舵機A
    ServoY.setupServoMotor();  // 舵機Y
    ServoX.setupServoMotor();  // 舵機X
    ServoB.setupServoMotor();  // 舵機B
    setupN20Motor();       // N20 馬達
    
}

void loop() {
    ble.update();  // 必須更新狀態
    
    if (ble.isConnected()) {
        float lx = ble.getLeftX();
        float ly = ble.getLeftY();
        
        
        // ★★★ N20 馬達控制 ★★★
        controlN20Motor(ly, lx);
        
        // ★★★ 舵機控制 ★★★
        controlServoMotor();

        //按鈕顯示
        /*if (ble.getButtonA()) Serial.print(" | A ");
        if (ble.getButtonB()) Serial.print(" | B ");
        if (ble.getButtonX()) Serial.print(" | X ");
        if (ble.getButtonY()) Serial.print(" | Y ");
        if (ble.getLB()) Serial.print(" | LB ");
        if (ble.getRB()) Serial.print(" | RB ");
        if (ble.getDpadUp()) Serial.print(" | ↑ ");
        if (ble.getDpadDown()) Serial.print(" | ↓ ");
        if (ble.getDpadLeft()) Serial.print(" | ← ");
        if (ble.getDpadRight()) Serial.print(" | → ");*/
        
        Serial.println();
    } else {
        Serial.print(".");  // 掃描中
        analogWrite(N20_ENA, 0);  // 未連線停止 N20
        delay(500);
    }
    
    delay(50);  // 20Hz
}
