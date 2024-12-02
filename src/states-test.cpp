#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>

float mag3(float a, float b, float c) {
    return sqrt(a * a + b * b + c * c);
}

SensorReadings readings;
int currentState = 0;

long long lastLoopTime = micros();

void setup() {
    Serial.begin(115200);
    Serial.println("States Test");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);
    IMU.begin();

    Serial.println("Initialized");
    Serial.println("Current State is 0 (Pad-Idle). Awaiting Liftoff...");

}


void loop() {

    IMU.readAcceleration(readings.ay, readings.ax, readings.az);

    switch (currentState) {
        case 0:
            flash(COLOR_BLUE);
            break;

        case 1:
            showColor(COLOR_GREEN);
            break;

        case 3:
            flash(COLOR_GREEN);
            break;

        case 4:
            showColor(COLOR_YELLOW);
            break;

        case 5:
            showColor(COLOR_BLUE);
            break;
    }

    if (millis() % 1000 < 10) {
        // --- State Transitions --- //

        // TOUCHDOWN: State 4 -> 5
        if (currentState == 4 && mag3(readings.ax, readings.ay, readings.az) <= 0.9) {
            Serial.println("Touchdown!");
            Serial.println("Current State has changed from 4 (Powered Descent) to 5 (Landing Idle).");
            Serial.println("Press RESET to restart.");
            currentState = 5;

        }

        // STAGE 2 IGNITION: State 3 -> 4
        if (currentState == 3 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
            Serial.println("Stage 2 Ignition!");
            Serial.println("Current State has changed from 3 (Coasting) to 4 (Powered Descent).");
            currentState = 4;
        }

        // STAGE 1 BURNOUT: State 1 -> 3
        if (currentState == 1 && mag3(readings.ax, readings.ay, readings.az) <= 0) {
            currentState = 3;
            Serial.println("Stage 1 Burnout!");
            Serial.println("Current State has changed from 1 (Powered Ascent) to 3 (Coasting).");
        }

        // LIFTOFF: State 0 -> 1
        if (currentState == 0 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
            currentState = 1;
            Serial.println("Liftoff!");
            Serial.println("Current State has changed from 0 (Pad-Idle) to 1 (Powered Ascent).");
        }
        delay(20);

    }
    lastLoopTime = micros();


}