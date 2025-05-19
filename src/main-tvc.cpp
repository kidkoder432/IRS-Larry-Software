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

enum LaunchStates {
    IDLE,
    LAUNCHING,
    LAUNCHED,
    ABORT
};

LaunchStates launchState = IDLE;
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
    // rocket.initSD();
    // rocket.printMessage("SD card initialized!");
    // rocket.getSdInfo();
    // rocket.initLogs();
    // rocket.printMessage("Data logging initialized!");

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

    switch (launchState) {
        case IDLE:
            showColor(COLOR_GREEN);
            recvOneChar();
            if (newCommand) {
                switch (receivedChar) {
                    case 'L':

                        rocket.printMessage("Launch sequence initiated!");
                        rocket.logMessage("Launch sequence initiated!");
                        rocket.printMessage("Performing pre-launch tasks.");
                        rocket.logMessage("Performing pre-launch tasks.");

                        rocket.initAngles();
                        rocket.tvc.lock();

                        rocket.printMessage("Calibrating Sensors");
                        rocket.calibrateAndLog();

                        rocket.setLogSpeed(MEDIUM);
                        rocket.setState(76);
                        launchState = LAUNCHING;
                        break;
                    case 'A':
                        rocket.printMessage("Launch aborted!");
                        launchState = ABORT;
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
                rocket.printMessage(dir.x, false);
                rocket.printMessage(", ", false);
                rocket.printMessage(dir.y, false);
                rocket.printMessage(", ", false);
                rocket.printMessage(dir.z);
            }
            break;
        case LAUNCHING:
            recvOneChar();
            if (newCommand) {

                // Abort with any key
                launchState = ABORT;
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
                recvOneChar();
                if (newCommand) {
                    rocket.printMessage("Launch aborted!");
                    rocket.logMessage("Launch aborted!");
                    newCommand = false;

                    rocket.finish();

                }

                if (millis() % 1000 < 40) {
                    rocket.printMessage("Launching in T - ", false);
                    rocket.printMessage((int64_t)launchTimer, false);
                    rocket.printMessage(" seconds! Press any key + ENTER to abort the launch.");

                    launchTimer--;
                    delay(41);

                }
            }
            else {
                rocket.printMessage("Launch sequence complete!");
                rocket.logMessage("Launch sequence complete!");

                rocket.printMessage("BLE will be DISABLED for the launch!");

                rocket.setLogSpeed(FAST);
                rocket.firePyro1();
                rocket.initAngles();

                rocket.disableCompl();
                rocket.bleOn = false;

                launchState = LAUNCHED;
                rocket.setState(1);
            }

            break;

        case LAUNCHED:
            // no special operations here
            break;

        case ABORT:
            rocket.printMessage("Flight or launch sequence aborted!");
            rocket.logMessage("Flight or launch sequence aborted!");
            rocket.firePyro2();  // Launch parachute
            rocket.abort();


    }

    int currentState = rocket.getCurrentState();

    switch (currentState) {
        case 0:     // Pad-Idle
            rocket.tvc.lock();
            break;
        case 1:     // Powered Ascent
            rocket.tvc.unlock();
            break;
        case 2:     // Coast
            rocket.tvc.lock();
            break;
        case 4:     // Touchdown
            rocket.logMessage("Touchdown confirmed. We are safe on Earth!");
            rocket.finish();
            break;
    }

    SensorReadings readings = rocket.getReadings();
    // --- State Transitions --- //

    // ABORT
    Vec3D dir = rocket.getDir();
    if ((abs(dir.x) >= 45 || abs(dir.z) >= 45) && launchState != IDLE) {
        launchState = ABORT;
    }

    // TOUCHDOWN: State 2 -> 4
    if (currentState == 2 && isBetween(mag3(readings.ax, readings.ay, readings.az), 0.9, 1.1)) {
        if (millis() - lastStepMs > 100) {
            onGroundSteps++;
            rocket.printMessage("On ground for ", false);
            rocket.printMessage((int64_t)onGroundSteps, false);
            rocket.printMessage(" /20 steps (", false);
            rocket.printMessage((float)onGroundSteps * 0.1f, false);
            rocket.printMessage(") sec.");
            lastStepMs = millis();
        }
    }

    if (onGroundSteps >= 20) {
        rocket.setState(4);
        rocket.parachute.cancel();
        rocket.logMessage("Touchdown confirmed. We are safe on Earth!");
        rocket.printMessage("Disarming pyros...");
        rocket.logMessage("Disarming pyros...");
        rocket.pyro1_motor.disarm();
        rocket.pyro2_land.disarm();

        stopTone();

        rocket.printMessage("Shutting down TVC...");
        rocket.logMessage("Shutting down TVC...");
        rocket.tvc.abort();

        rocket.setLogSpeed(MEDIUM);
        playLocatorSound();
    }

    // STAGE 1 BURNOUT: State 1 -> 2
    if (currentState == 1 && isBetween(mag3(readings.ax, readings.ay, readings.az), 0, 0.9)) {
        rocket.logMessage("Stage 1 burnout");
        rocket.logMessage("Coasting");
        rocket.logMessage("Deploying parachute");
        rocket.fireChutes();
        rocket.setState(2);
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