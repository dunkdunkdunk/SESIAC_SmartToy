#include <ESP32Servo.h>
#include <analogWrite.h>
#include <tone.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>

Servo leftServo;  
Servo rightServo;

const int leftServoPin = 18;  
const int rightServoPin = 19;

void setup() {
    Serial.begin(115200);
    delay(1000);  // Allow time for Serial Monitor to start

    Serial.println("Initializing servos...");

    // Attach servos
    leftServo.attach(leftServoPin);
    rightServo.attach(rightServoPin);

    leftServo.write(90);
    rightServo.write(90);

    Serial.println("Servos initialized.");
}

void loop() {
    if (Serial.available()) {
        char command = Serial.read();
        Serial.print("Received: ");
        Serial.println(command);

        if (command == 'L') {
            Serial.println("Activating left servo...");
            leftServo.write(45);
            delay(300);
            leftServo.write(90);
        } 
        
        else if (command == 'R') {
            Serial.println("Activating right servo...");
            rightServo.write(135);
            delay(300);
            rightServo.write(90);
        }
    }
}
