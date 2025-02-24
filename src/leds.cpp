#include <leds.h>
#include <WiFiNINA.h>

void setup() {
    Serial.begin(115200);
    pinMode(LEDR_PIN, OUTPUT);
    pinMode(LEDG_PIN, OUTPUT);
    pinMode(LEDB_PIN, OUTPUT);
    showColor(COLOR_OFF);
}

void loop() {

    flash(COLOR_BLUE, 1000);
    delay(1);
}

