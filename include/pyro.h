// PYROTECHNIC CHANNELS

#ifndef PYRO_H
#define PYRO_H
#include "Arduino.h" 


// Pins
const int PYRO_LANDING_MOTOR_IGNITION = 4;
const int PYRO_LANDING_LEGS_DEPLOY = 3;

void fire_pyro_test(int pin) {
    digitalWrite(pin, 1);
    delay(1000);
    digitalWrite(pin, 0);
}

class PyroChannel {
public:
    int pin;
    long fireTime = 0;
    bool activeLow = false;
    bool oneShot = false;


    PyroChannel() {
    #if USE_RP2040
        this->pin = 27;
    #else
        this->pin = LEDR;
    #endif
        this->fireTime = 0;
        this->activeLow = false;
        this->oneShot = false;

    }

    PyroChannel(int pin, long fireTime) {
        this->pin = pin;
        this->fireTime = fireTime;
        this->activeLow = false;
        this->oneShot = false;

    }
    PyroChannel(int pin, long fireTime, bool activeLow) {
        this->pin = pin;
        this->fireTime = fireTime;
        this->activeLow = activeLow;
        this->oneShot = false;

    }

    PyroChannel(int pin, long fireTime, bool activeLow, bool oneShot) {
        this->pin = pin;
        this->fireTime = fireTime;
        this->activeLow = activeLow;
        this->oneShot = oneShot;

    }

    void begin() {
        pinMode(this->pin, OUTPUT);
        if (this->activeLow) {
            digitalWrite(this->pin, 1);
        }
        else {
            digitalWrite(this->pin, 0);
        }
    }

    void arm() {
        if (activeLow) {
            digitalWrite(this->pin, 1);
        }
        else {
            digitalWrite(this->pin, 0);
        }
        this->isArmed = true;
    }

    void disarm() {
        this->stop();
        this->isArmed = false;
    }

    void fire() {
        if (!this->isFiring && this->isArmed) {
            this->isFiring = true;
            this->startTime = millis();
        }
        if (!this->isArmed) {
            Serial.println("ERROR: Pyro channel not armed!");
        }
    }

    void stop() {
        this->isFiring = false;
        if (this->activeLow) {
            digitalWrite(pin, 1);
        }
        else {
            digitalWrite(pin, 0);
        }
    }

    void update() {
        if (this->isFiring && this->isArmed) {
            if (millis() - this->startTime > this->fireTime) {
                if (activeLow) {
                    digitalWrite(this->pin, 1);
                }
                else {
                    digitalWrite(this->pin, 0);
                }
                this->isFiring = false;
            }
            else {
                if (this->oneShot) {
                    this->isArmed = false;
                }
                if (this->activeLow) {
                    digitalWrite(this->pin, 0);
                }
                else {
                    digitalWrite(this->pin, 1);
                }
            }
        }
        else {
            if (activeLow) {
                digitalWrite(this->pin, 1);
            }
            else {
                digitalWrite(this->pin, 0);
            }
        }

    }

private:
    long long startTime = 0;
    bool isFiring = false;
    bool isArmed = false;

};

#endif