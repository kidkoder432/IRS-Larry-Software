#include <leds.h>
#include <WiFiNINA.h>

Color colors[] = {
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIGHTGREEN,
    COLOR_GREEN,
    COLOR_LIGHTBLUE,
    COLOR_BLUE,
    COLOR_PURPLE,
    COLOR_PINK,
    COLOR_WHITE,
    COLOR_OFF
};

void setup() {
    Serial.begin(115200);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);
}

long lastMillis = 0;
int count;

void loop() {
    for (int i = 0; i < sizeof(colors) / sizeof(Color); ++i) {
        while (millis() - lastMillis < 2000) {
            flash(colors[i], 150);
            delay(1);
        }
        lastMillis = millis();
        delay(3);
    }

    // byte last3Bits = ~((count & 0x7) ^ 0x7);
    // digitalWrite(LEDR, (last3Bits & 0x1) ? HIGH : LOW);
    // digitalWrite(LEDG, ((last3Bits >> 1) & 0x1) ? HIGH : LOW);
    // digitalWrite(LEDB, ((last3Bits >> 2) & 0x1) ? HIGH : LOW);
    // Serial.print("Count: ");
    // Serial.print(count);
    // Serial.print(" Last 3 bits: ");
    // Serial.print((count & 0x7), BIN);
    // Serial.print(" ^ 0x7: ");
    // Serial.print(((count & 0x7) ^ 0x7), BIN);
    // Serial.print(" R: ");
    // Serial.print((last3Bits & 0x1) ? HIGH : LOW);
    // Serial.print(" G: ");
    // Serial.print(((last3Bits >> 1) & 0x1) ? HIGH : LOW);
    // Serial.print(" B: ");
    // Serial.println(((last3Bits >> 2) & 0x1) ? HIGH : LOW);
    // count += 1;
    // count = count % 8;
    
    // delay(500);


}

