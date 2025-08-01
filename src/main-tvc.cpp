// Test all components and features

#include <rocket.h>

float isBetween(float x, float a, float b) { return a <= x && x <= b; }

Rocket rocket;

char receivedChar;
bool newCommand = false;

#if USE_BLE 
HardwareBLESerial& bleSerial = rocket.getBle();
#endif

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#if USE_BLE
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#endif
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void printTvc() {
    Vec2D tvcOut = rocket.tvc.getAngle();
    rocket.printMessage("TVC X: ", false);
    rocket.printMessage(tvcOut.x, false);
    rocket.printMessage(" Y: ", false);
    rocket.printMessage(tvcOut.y);



}
const int SEC_TO_LAUNCH = 21;
int launchTimer = SEC_TO_LAUNCH;

unsigned long stage1BurnoutTime = 0;

int onGroundSteps = 0;
unsigned long lastStepMs = 0;

bool dirOutLock = true;

char HELP_STR[] =
R"(Welcome!
This is the LAUNCH CONTROL SOFTWARE for the TAX COLLECTOR
Use this software to control launch, run specific routines, or abort the launch.
Commands:
L: Launch
A: Abort
C: Calibrate Sensors

Y: Test TVC
D: Unlock TVC
F: Lock TVC

V: Nudge X -
B: Nudge X +
N: Nudge Y -
M: Nudge Y +

Q: Reset Angles
O: Print Orientation
X: Cancel Printing

H: Help
)";

void setup() {

    // Setup basic interfaces
    rocket.initSerial();
    delay(2000);

    rocket.initBuzzer();
    rocket.printMessage("Buzzer initialized!");

    rocket.initLeds();
    rocket.printMessage("LEDs initialized!");

    // Setup SD card, config and data logging
    rocket.initSD();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
    rocket.printMessage("Data logging initialized!");

    rocket.initConfig();
    rocket.printMessage("Config initialized!");

    // Init sensors and angles
    rocket.setupSensors();
    rocket.calibrateAndLog();

#if USE_BLE
    rocket.initBle();
    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 && !override) {
        flash(COLOR_BLUE);
        delay(2);
        recvOneChar();
        if (newCommand) {
            switch (receivedChar) {
                case 'Z':
                    rocket.bleOn = false;
                    override = true;
                    break;
                default:
                    override = false;
                    break;
            }
        }

        if (millis() % 1000 < 40) {
            rocket.printMessage("BLE not connected, waiting...");
            waits++;
            delay(41);
            showColor(COLOR_OFF);

        }
    }

    if (bleSerial) {
        rocket.bleOn = true;
        rocket.printMessage("HardwareBLESerial central device connected!");
        rocket.logMessage("HardwareBLESerial central device connected!");
    }
    else {
        rocket.bleOn = false;
        rocket.printMessage("BLE not connected! Using USB serial...");
        rocket.logMessage("WARN: BLE not connected! Using USB serial...");

    }
#endif

    rocket.initAngles();
    rocket.printMessage("Sensors and angles initialized!");

    // init hardware
    rocket.printMessage("LEDs initialized!");
    rocket.initTvc();
    rocket.printMessage("TVC initialized!");

    // init pyros and complete only after other inits succeed
    rocket.initPyros();
    rocket.printMessage("Pyros initialized!");

    rocket.initChutes();
    rocket.printMessage("Chutes initialized!");

    rocket.finishSetup();

    rocket.printMessage("Setup complete!");
    rocket.printMessage(HELP_STR);
}

void loop() {

#if USE_BLE
    if (rocket.bleOn) rocket.updateBle();
#endif

    switch (rocket.getState()) {

        case FS_READY:
            recvOneChar();
            if (newCommand) {
                switch (receivedChar) {
                    case 'L':

                        rocket.printMessage("Launch sequence initiated!");
                        rocket.logMessage("Launch sequence initiated!");
                        rocket.printMessage("Performing pre-launch tasks.");
                        rocket.logMessage("Performing pre-launch tasks.");

                        rocket.printMessage("Calibrating Sensors");
                        rocket.calibrateAndLog();

                        rocket.setLogSpeed(DLS_MEDIUM);

                        rocket.tvc.unlock();
                        rocket.printMessage("TVC unlocked!");

                        rocket.setState(FS_LAUNCHING);
                        break;
                    case 'A':
                        rocket.printMessage("Flight or launch sequence aborted!");
                        rocket.logMessage("Flight or launch sequence aborted!");
                        rocket.abort();
                        rocket.setState(FS_ABORT);
                        break;
                    case 'C':
                        rocket.calibrateAndLog();
                        break;

                    case 'Y':
                        rocket.printMessage("Running TVC test");
                        rocket.tvc.testRoutine();
                        break;
                    case 'D':
                        rocket.printMessage("Unlocking TVC");
                        rocket.tvc.unlock();
                        break;
                    case 'F':
                        rocket.printMessage("Locking TVC");
                        rocket.tvc.lock();
                        break;

                    case 'V':
                        rocket.printMessage("Nudging X -");
                        rocket.tvc.nudge(1);
                        printTvc();
                        break;
                    case 'B':
                        rocket.printMessage("Nudging X +");
                        rocket.tvc.nudge(0);
                        printTvc();
                        break;
                    case 'N':
                        rocket.printMessage("Nudging Y -");
                        rocket.tvc.nudge(3);
                        printTvc();
                        break;
                    case 'M':
                        rocket.printMessage("Nudging Y +");
                        rocket.tvc.nudge(2);
                        printTvc();
                        break;

                    case 'Q':
                        rocket.printMessage("Resetting Angles");
                        rocket.initAngles();
                        break;
                    case 'O':
                        rocket.printMessage("Orientation: ");
                        dirOutLock = false;
                        break;
                    case 'X':
                        dirOutLock = true;
                        break;
                    case 'H':
                        rocket.printMessage(HELP_STR);
                        break;
                    case 'P':
                        rocket.printMessage("Time per loop: ", false);
                        rocket.printMessage(rocket.deltaTime);

                        rocket.printMessage("Loop rate: ", false);
                        rocket.printMessage(1 / rocket.deltaTime);
                        break;
                    default:
                        rocket.printMessage("Invalid command!");
                        break;
                }
            }

            if (!dirOutLock) {
                Vec3D dir = rocket.getDir();
                float roll = dir.x;
                float pitch = dir.y;
                float yaw = dir.z;
                rocket.printMessage(roll, false);
                rocket.printMessage(" ", false);
                rocket.printMessage(pitch, false);
                rocket.printMessage(" ", false);
                rocket.printMessage(yaw);
            }
            break;
        case FS_LAUNCHING:
            recvOneChar();
            if (newCommand) {

                rocket.setState(FS_ABORT);
                rocket.printMessage("Flight or launch sequence aborted!");
                rocket.logMessage("Flight or launch sequence aborted!");
                rocket.abort();
                break;
            }

            if (launchTimer > 0) {

                if (launchTimer > 12) {
                    flash(COLOR_YELLOW, 500);
                    beepTone(400, 500);

                }
                else if (launchTimer > 5) {
                    flash(COLOR_RED, 500);
                    beepTone(400, 250);
                }
                else {
                    flash(COLOR_RED, 250);
                    playToneForever(400);
                }

                if (millis() % 1000 < 40) {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "Launching in T - %02d seconds! Press any key + Enter to abort. ", launchTimer);
                    rocket.printMessage(buf);
                    rocket.logMessage(buf);
                    launchTimer--;
                    delay(41);

                }
            }
            else {
                rocket.printMessage("Launch sequence complete!");
                rocket.logMessage("Launch sequence complete!");

                rocket.setLogSpeed(DLS_FAST);
                rocket.initAngles();
                rocket.disableCompl();
                rocket.tvc.reset();
                
                rocket.printMessage("Rocket armed!");
                rocket.printMessage("Awaiting ignition...");
                rocket.setState(FS_ARMED);
            }

            break;

        case FS_ARMED:
            recvOneChar();
            if (newCommand) {
                rocket.setState(FS_ABORT);
                rocket.printMessage("Flight or launch sequence aborted!");
                rocket.logMessage("Flight or launch sequence aborted!");
                rocket.abort();
            }
            break;
        case FS_THRUST:
            // no special operations here
            break;
        case FS_COASTING:
            // no special operations here
            break;

        case FS_TOUCHDOWN:
            recvOneChar();
            playLocatorSound();
            if (millis() % 2000 < 40) {
                rocket.printMessage("Press any key to shut down the rocket and all systems. ");
                delay(41);
            }
            if (newCommand) {
                rocket.setState(FS_SHUTDOWN);
            }
            break;

        case FS_ABORT:
            playAbortSound();
            if (millis() % 2000 < 40) {
                rocket.printMessage("Press any key to shut down the rocket and all systems. ");
                delay(41);
            }

            recvOneChar();
            if (newCommand) {
                rocket.setState(FS_SHUTDOWN);
            }
            break;

        case FS_SHUTDOWN:
            rocket.finish();
            break;
    }

    int flightState = rocket.getState();

    SensorReadings readings = rocket.getReadings();
    // --- State Transitions --- //

    // ABORT
    Vec3D dir = rocket.getDir();
    // if ((abs(dir.y) >= 45 || abs(dir.z) >= 45) && flightState != FS_READY && flightState != FS_ABORT && flightState != FS_TOUCHDOWN) {
    //     rocket.setState(FS_ABORT);
    //     rocket.printMessage("Flight or launch sequence aborted!");
    //     rocket.logMessage("Flight or launch sequence aborted!");
    //     rocket.abort();
    // }

    // TOUCHDOWN: State COASTING -> TOUCHDOWN
    if (flightState == FS_COASTING && isBetween(mag3(readings.ax, readings.ay, readings.az), 0.9, 1.1)) {
        if (millis() - lastStepMs > 100) {
            onGroundSteps++;
            char buf[128];
            snprintf(buf, sizeof(buf), "On ground for %d/%d steps (%.1f) sec.", onGroundSteps, 20, (float)onGroundSteps * 0.1f);
            rocket.printMessage(buf);
            lastStepMs = millis();
        }
    }

    if (onGroundSteps >= 20 && flightState == FS_COASTING) {
        rocket.setState(FS_TOUCHDOWN);
        rocket.parachute.cancel();
        rocket.parachute.disarm();
        rocket.logMessage("Touchdown confirmed. We are safe on Earth!");
        rocket.printMessage("Disarming pyros...");
        rocket.logMessage("Disarming pyros...");
        rocket.pyro1_motor.disarm();
        rocket.pyro2_land.disarm();

        rocket.printMessage("Shutting down TVC...");
        rocket.logMessage("Shutting down TVC...");
        rocket.tvc.abort();

        rocket.setLogSpeed(DLS_MEDIUM);
    }

    // STAGE 1 BURNOUT: State THRUST -> COASTING
    if (flightState == FS_THRUST && isBetween(mag3(readings.ax, readings.ay, readings.az), 0, 0.5)) {
        rocket.logMessage("Stage 1 burnout");
        rocket.logMessage("Coasting");
        rocket.logMessage("Deploying parachute on timer");
        rocket.fireChutes();
        rocket.setState(FS_COASTING);
    }

    // LIFTOFF: State ARMED -> THRUST
    if (flightState == FS_ARMED && isBetween(mag3(readings.ax, readings.ay, readings.az), 1.2, 5)) {
        rocket.logMessage("Liftoff");
        rocket.setState(FS_THRUST);
    }

    // constrain to 100hz
    rocket.updateTime();

    // Update spatial data
    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();

    // Update hardware
    rocket.updateTvc();
    rocket.updatePyros();
    rocket.updateChutes();

    // Update logging
    rocket.updateDataLog();

    rocket.updateStateLeds();
}