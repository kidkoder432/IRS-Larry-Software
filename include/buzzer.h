#include <Arduino.h>

void playConstantTone(int freq, int duration) {
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, freq, duration);
}

void playToneForever(int freq) {
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, freq);
}

void stopTone() {
    noTone(BUZZER_PIN);
}

void beepTone(int freq) {
    if ((millis() / 500) == 1) {
        tone(BUZZER_PIN, freq);
    }
    else {
        noTone(BUZZER_PIN);
    }

}

void beepTone(int freq, int delay) {
    if ((millis() / delay) == 1) {
        tone(BUZZER_PIN, freq);
    }
    else {
        noTone(BUZZER_PIN);
    }
}

void playStartupSound() {
    int notes[4] = { 415, 523, 622, 831 }; // G#4 C5 D#5 G#5
    for (int i = 0; i < 4; i++) {
        tone(BUZZER_PIN, notes[i], 215);
        delay(250);
    }
    noTone(BUZZER_PIN);
}

void playAbortSound() {
    beepTone(415, 250);
}

void playShutdownSound() {
    int notes[4] = { 813, 622, 523, 415 }; // G#5 D#5 C5 G#4
    for (int i = 0; i < 4; i++) {
        tone(BUZZER_PIN, notes[i], 215);
        delay(250);
    }
}

void playLocatorSound() {
    beepTone(415, 1000);
}