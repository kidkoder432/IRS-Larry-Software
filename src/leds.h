// LED CONTROLLER

#include <Arduino.h>

const int DELAY_MS = 300;

struct Color {
    char r, g, b;
    Color(char r, char g, char b) : r(r), g(g), b(b) {}
};

// Basic Colors
struct Color COLOR_RED(255, 0, 0);
struct Color COLOR_GREEN(0, 255, 0);
struct Color COLOR_ORANGE(255, 128, 0);
struct Color COLOR_YELLOW(255, 200, 0);
struct Color COLOR_BLUE(0, 100, 255);
struct Color COLOR_PURPLE(128, 0, 255);
struct Color COLOR_WHITE(255, 255, 255);
struct Color COLOR_OFF(0, 0, 0);

void showColor(Color c) {
    analogWrite(LEDR, 255 - c.r);
    analogWrite(LEDG, 255 - c.g);
    analogWrite(LEDB, 255 - c.b);
}

void off() {
    showColor(COLOR_OFF);
}

void flash(Color c) {
    if ((millis() / DELAY_MS) & 1 == 1) {
        showColor(c);
    }
    else {
        showColor(COLOR_OFF);
    }
}

void LED_STATES(int currentState) {
    switch (currentState) {
        case 0:
            flash(COLOR_BLUE);
        case 1:
            showColor(COLOR_GREEN);
        case 2:
            break; // Not used (for now)
        case 3:
            flash(COLOR_YELLOW);
        case 4:
            showColor(COLOR_YELLOW);
        case 5:
            showColor(COLOR_BLUE);
    }
}
