// PYROTECHNIC CHANNELS

#include <Serial.h>
#include "Arduino.h" 
#include <FireTimer.h>


// Pins
#define PYRO_LANDING_MOTOR_IGNITION 2
#define PYRO_LANDING_LEGS_DEPLOY 3


void fire_pyro(FireTimer t, int pin)
{
    if (t.fire(false)) {
        digitalWrite(pin, 0);
    }
    else {
        digitalWrite(pin, 1);
    }
}