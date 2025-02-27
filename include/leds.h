// LED CONTROLLER

#include <Arduino.h>

const int DELAY_MS = 100;

struct Color {
    int r, g, b;
    Color(int r, int g, int b) : r(r), g(g), b(b) {}
};

// Basic Colors
struct Color COLOR_RED(255, 0, 0);
struct Color COLOR_ORANGE(255, 128, 0);
struct Color COLOR_YELLOW(255, 255, 0);
struct Color COLOR_LIGHTGREEN(128, 255, 0);
struct Color COLOR_GREEN(0, 255, 0);
struct Color COLOR_LIGHTBLUE(0, 128, 255);
struct Color COLOR_BLUE(0, 0, 255);
struct Color COLOR_PURPLE(128, 0, 255);
struct Color COLOR_PINK(255, 0, 255);
struct Color COLOR_WHITE(255, 255, 255);
struct Color COLOR_OFF(0, 0, 0);

void showColor(Color c) {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    if (c.r == 255) {
        digitalWrite(LEDR, HIGH);
    }
    else if (c.r == 0) {
        digitalWrite(LEDR, LOW);
    }
    else {
        analogWrite(LEDR, 255 - c.r);
    }
    if (c.g == 255) {
        digitalWrite(LEDG, HIGH);
    }
    else if (c.g == 0) {
        digitalWrite(LEDG, LOW);
    }
    else {
        analogWrite(LEDG, 255 - c.g);
    }
    if (c.b == 255) {
        digitalWrite(LEDB, HIGH);
    }
    else if (c.b == 0) {
        digitalWrite(LEDB, LOW);
    }
    else {
        analogWrite(LEDB, 255 - c.b);
    }
}

void off() {
    showColor(COLOR_OFF);
}

void flash(Color c) {
    if (((millis() / DELAY_MS) & 1) == 1) {
        showColor(c);
    }
    else {
        showColor(COLOR_OFF);
    }
}

void flash(Color c, int DELAY_MS) {
    if (((millis() / DELAY_MS) & 1) == 1) {
        showColor(c);
    }
    else {
        showColor(COLOR_OFF);
    }
}

void flash(Color c1, Color c2) {
    if (((millis() / DELAY_MS) & 1) == 1) {
        showColor(c1);
    }
    else {
        showColor(c2);
    }
}

void flash(Color c1, Color c2, int DELAY_MS) {
    if (((millis() / DELAY_MS) & 1) == 1) {
        showColor(c1);
    }
    else {
        showColor(c2);
    }
}

