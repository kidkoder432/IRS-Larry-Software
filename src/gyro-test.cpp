#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>


double yaw, pitch;
SensorReadings readings, prevReadings;
Vec2D dir;
Biases biases;

bool newCommand = false;
char receivedChar;

void setup() {
    Serial.begin(115200);
    delay(200);

    initSensors();

    pinMode(LED_BUILTIN, OUTPUT);
    showColor(COLOR_OFF);
    biases = calibrateSensors();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);

    yaw = 0;
    pitch = 0;

    pinMode(3, INPUT_PULLUP);

}

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        newCommand = true;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

long long lastLoopTime = micros();
void loop() {
    // --- Read Sensors --- //
    // IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    readSensors(readings, biases);

    recvOneChar();

    if (newCommand) {
        switch (receivedChar) {
            case 'Q':
                yaw = 0;
                pitch = 0;
                dir = Vec2D(0, 0);
                break;
        }

        newCommand = false;
    }

    // IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    dir = get_angles(readings, prevReadings, dir, DELTA_TIME);

    yaw = dir.x;
    pitch = dir.y;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(pitch);

    if (abs(yaw) >= 15) {
        showColor(COLOR_RED);
        if (abs(pitch) >= 15) {
            showColor(COLOR_PURPLE);
        }
    }

    else {
        showColor(COLOR_GREEN);
        if (abs(pitch) >= 15) {
            showColor(COLOR_LIGHTBLUE);
        }
    }


    while ((micros() - lastLoopTime) < 990) {}
    DELTA_TIME = (micros() - lastLoopTime) / 1000000.;
    lastLoopTime = micros();
    prevReadings = readings;
}