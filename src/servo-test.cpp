#include <Arduino.h>
#include <Servo.h> 
#include <orientation.h>

Servo myservo;  // create servo object to control a servo 
String readString;

void setup() {
    Serial.begin(9600);
    delay(2000);
    myservo.attach(6);
    Serial.println(myservo.read());
    Serial.println("servo-test"); // so I can keep track of what is loaded
}

const int MIN = 80;

void loop() {

    while (Serial.available()) {
        char c = Serial.read();  // gets one byte from serial buffer
        readString += c; // makes the String readString
        delay(2);  // slow looping to allow buffer to fill with next character
    }

    if (readString.length() > 0) {
        Serial.println(readString);  // so you can see the captured String 
        int n = readString.toInt();  // convert readString into a number
        Serial.println(myservo.read());
        myservo.write(n);
        Serial.println(myservo.read());
        readString = "";
    }
}