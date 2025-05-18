#include <leds.h>

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

float absx(float x) { return x < 0 ? -x : x; }

void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    Serial.begin(115200);
}

float yaw = 19.9;
float pitch;
long ms;
int i;

char* colorNames[] = { "red", "orange", "yellow", "lightgreen", "green", "lightblue", "blue", "purple", "pink", "white", "off" };

void loop() {

    i = 0;
    for (Color color : colors) {
        ms = millis();
        while (millis() - ms < 1000) {
            showColor(color);
            Serial.print(millis());
            Serial.print(" ");
            Serial.println(colorNames[i]);
            delay(1);
        }
        i++;

    }
}
