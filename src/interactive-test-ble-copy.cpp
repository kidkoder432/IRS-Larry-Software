// Test all components and features

#include <routines.h>

#define SD_ATTACHED 1

Rocket rocket;

HardwareBLESerial& bleSerial = rocket.getBle();

char receivedChar;
bool newCommand = false;
bool dirOutLock = true;
bool sensorOutLock = true;
bool experimentMode = false;

const char HELP_STR[] =
R"(Input commands to test the following:
U: Unlock the TVC
L: Lock the TVC
K: Toggle LEDs
C: Calibrate the Gyro
O: Print Orientation
A: Print Altitude
R: Print Sensor Readings
D: Toggle Data Logging
X: Cancel Printing
G: Activate Pyro 1 (Motor Ignition)
T: Activate Pyro 2 (Landing Legs Deploy)
S: Read SD Card
Q: Reset Angles
P: Show Performance Metrics
B: Edit Config (INOP)
E: Toggle Experiment/Test Mode
Z: Switch Bluetooth to USB (override)
H: Help)";

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void setup() {

    // Setup basic interfaces
    rocket.initSerial();
    delay(2000);


#if SD_ATTACHED
    // Setup SD card, config and data logging
    rocket.initSd();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
    rocket.printMessage("Data logging initialized!");
    rocket.initConfig();
    rocket.printMessage("Config initialized!");

    rocket.initBle();

    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 and !override) {
        flash(COLOR_BLUE);
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
            delay(50);
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

    // Init sensors and angles
    rocket.setupSensors();
    rocket.calibrateAndLog();
    rocket.initAngles();
    rocket.printMessage("Sensors and angles initialized!");

    // init hardware
    rocket.initLeds();
    rocket.printMessage("LEDs initialized!");
    rocket.initTvc();
    rocket.printMessage("TVC initialized!");

    // init pyros and complete only after other inits succeed
    rocket.initPyros();
    rocket.printMessage("Pyros initialized!");
    rocket.finishSetup();

    rocket.printMessage("Setup complete!");
    rocket.printMessage("Welcome to the Tax Collector Test Suite");
    rocket.printMessage("This script tests all components and features of the rocket,");
    rocket.printMessage("including sensors, LEDs, TVC, and pyro channels.");
    rocket.printMessage(HELP_STR);
}

void loop() {

    recvOneChar();
    if (rocket.bleOn) rocket.updateBle();

    rocket.updateTvc();
    rocket.updateSensors();
    rocket.updateAngles();
    // rocket.updateAltVel();
    // rocket.updateState();
    rocket.updatePyros();
    rocket.updateLeds();
    rocket.logData();

    if (newCommand == true) {
        switch (receivedChar) {
            case 'U':
                rocket.setTvc(false);
                break;
            case 'L':
                rocket.setTvc(true);
                break;
            case 'K':
                rocket.printMessage("Toggling LEDs");
                rocket.ledsOn = !rocket.ledsOn;
                break;
            case 'C':
                rocket.calibrateAndLog();
                break;
            case 'O':
                msgPrint(rocket.bleOn, bleSerial, "Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                rocket.updateAltVel();
                rocket.printMessage("Altitude: ", false);
                rocket.printMessage(rocket.getAlt());
                break;
            case 'R':
                rocket.printMessage("Sensor Readings: ");
                dirOutLock = true;
                sensorOutLock = false;
                break;
            case 'X':
                sensorOutLock = true;
                dirOutLock = true;
                break;
            case 'G':
                rocket.printMessage("Pyro 1: Motor Ignition");
                rocket.firePyro1();
                // fire_pyro_test(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                rocket.printMessage("Pyro 2: Landing Legs Deploy");
                rocket.firePyro2();
                // fire_pyro_test(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                rocket.printMessage("Reading SD Card Info");
                rocket.getSdInfo();
                break;
            case 'Q':
                rocket.printMessage("Resetting Angles");
                rocket.resetAngles();
                break;
            case 'H':
                rocket.printMessage(HELP_STR);
                break;

            case 'D':
                rocket.doLog = !rocket.doLog;
                if (rocket.doLog) {
                    rocket.printMessage("Data Logging Enabled, opening logs");
                    rocket.initLogs();
                }
                else {
                    rocket.printMessage("Data Logging Disabled, saving logs");
                    rocket.cleanupLogs();

                }
                break;
            case 'P':
                rocket.printMessage("Time per loop: ", false);
                rocket.printMessage(rocket.deltaTime);

                rocket.printMessage("Loop rate: ", false);
                rocket.printMessage(1 / rocket.deltaTime);

                break;

            case 'E':
                experimentMode = !experimentMode;
                if (experimentMode) {
                    rocket.currentState = 127;
                    rocket.printMessage("Experiment Mode ON");
                    rocket.tvc.unlock();
                    if (!rocket.doLog) rocket.setDataLog(true);
                }
                else {
                    rocket.currentState = 42;
                    rocket.printMessage("Experiment Mode OFF");
                    rocket.tvc.lock();
                }
                break;

            default:
                rocket.printMessage("Invalid command");
                break;
        }
        newCommand = false;
    }

    if (!sensorOutLock) {
        SensorReadings readings = rocket.getReadings();
        rocket.printMessage("Accelerometer: ", false);
        rocket.printMessage(readings.ax, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.ay, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.az);

        rocket.printMessage("Gyroscope: ", false);
        rocket.printMessage(readings.gx, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.gy, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.gz);
    }

    if (!dirOutLock) {
        Vec3D dir = rocket.getDir();
        float yaw = dir.x;
        float pitch = dir.y;
        float roll = dir.z;
        rocket.printMessage(yaw, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(pitch, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(roll);
    }

    rocket.updateTime();
}

