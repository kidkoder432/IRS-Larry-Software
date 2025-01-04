#include <Arduino.h>
#include <Servo.h> 
#include <pins.h>
#include <orientation.h>

Servo xservo, yservo;  // create servo object to control a servo 
String readString;

void setup() {
    Serial.begin(9600);
    delay(2000);
    xservo.attach(TVC_X_PIN);
    yservo.attach(TVC_Y_PIN);
    Serial.println("servo-test"); // so I can keep track of what is loaded
}

const int XDEF = 90;
const int YDEF = 90;
int mode = 0;

void loop() {

    readString = Serial.readStringUntil('\n');
    if (readString.length() > 0) {
        Serial.println("eeee");
        if (readString == "x") {
            mode = 0;
        } else if (readString == "y") {
            mode = 1;
        } else if (readString == "r") {
            xservo.write(XDEF);
            yservo.write(YDEF);
        } else {
            int n = readString.toInt();  // convert readString into a number

            if (mode == 0) {
                xservo.write(n);
            } else if (mode == 1) {
                yservo.write(n);
            }

        }
       
        readString = "";
    }
}