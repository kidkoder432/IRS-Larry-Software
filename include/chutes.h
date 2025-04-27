#include <pyro.h>
#include <config.h>

class Parachute {
public:

    Parachute(int pin) {
        parachuteChannel = PyroChannel(pin, 1000, false, false);
    }

    void config(Config config) {
        parachuteChannel.fireTime = (long)round(config["PARACHUTE_FIRE_TIME"]);
        parachuteChannel.oneShot = config["PARACHUTE_ONE_SHOT"] > 0;

        delayMs = (long)round(config["PARACHUTE_BURNOUT_DELAY"]);

    }

    void arm() {
        parachuteChannel.arm();
    }

    void deployTimer() {
        if (!timerOn) {
            startTime = millis();
            timerOn = true;
        }
    }

    bool update() {
        parachuteChannel.update();

        if (timerOn && (millis() - startTime) > delayMs) {
            deploy();
            timerOn = false;
            return true;
        }
        return false;
    }

    void cancel() {
        timerOn = false;
        parachuteChannel.stop();
    }

    void disarm() {
        timerOn = false;
        parachuteChannel.disarm();
    }

    void deploy() {
        parachuteChannel.fire();
    }

private:
    PyroChannel parachuteChannel;
    unsigned long delayMs = 0;
    unsigned long startTime = 0;
    bool timerOn = false;

};