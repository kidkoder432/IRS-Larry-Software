// PYROTECHNIC CHANNELS

#include <Serial.h>
#include "Arduino.h" 
#include <FireTimer.h>


// Pins
#define PYRO_LANDING_MOTOR_IGNITION 4
#define PYRO_LANDING_LEGS_DEPLOY 3


void fire_pyro(FireTimer t, int pin) {
    if (!t.fire(false)) {
        digitalWrite(pin, 1);
    }
    else {
        digitalWrite(pin, 0);
    }
}


void fire_pyro_test(int pin) {
    digitalWrite(pin, 1);
    delay(1000);
    digitalWrite(pin, 0);
}