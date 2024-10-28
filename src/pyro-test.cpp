#include <Arduino.h>
#include <Serial.h>
#include <pyro.h>

PyroChannel pyro1 = PyroChannel(LED_BUILTIN, 1000);
PyroChannel pyro2 = PyroChannel(LEDR, 1000, true);

void setup() {
    Serial.begin(115200);
    delay(2000);

    pyro1.begin();
    pyro2.begin();

    pyro1.arm();
    pyro2.arm();

}

bool newCommand = false;
char receivedChar;

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        newCommand = true;
        return;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void loop() {
    recvOneChar();

    pyro1.update();
    pyro2.update();

    if (newCommand) {
        switch (receivedChar) {
            case 'A':
                Serial.println("Firing pyro 1 (LED_BUILTIN)...");
                pyro1.fire();
                Serial.println("Done.");
                break;
            case 'B':
                Serial.println("Firing pyro 2 (LEDR)...");
                pyro2.fire();
                Serial.println("Done.");
                break;
        }
        newCommand = false;
    }
}