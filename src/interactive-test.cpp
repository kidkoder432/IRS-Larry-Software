// Test all components and features

#include <rocket.h>

Rocket rocket;

#if USE_BLE 
HardwareBLESerial& bleSerial = rocket.getBle();
#endif

char receivedChar;
bool newCommand = false;
bool dirOutLock = true;
bool sensorOutLock = true;
bool experimentMode = false;

long currentMs;

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
S: SD Card Info
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

void setup() {

    // Setup basic interfaces
    rocket.initSerial();
    delay(2000);


    // Setup SD card, config and data logging
    rocket.initSD();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
    rocket.printMessage("Data logging initialized!");
    rocket.initConfig();
    rocket.printMessage("Config initialized!");

#if USE_BLE
    rocket.initBle();
    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 && !override) {
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

    rocket.initBuzzer();
    rocket.printMessage("Buzzer initialized!");

    rocket.finishSetup();

    rocket.printMessage("Setup complete!");
    rocket.printMessage("Welcome to the Tax Collector Test Suite");
    rocket.printMessage("This script tests all components and features of the rocket,");
    rocket.printMessage("including sensors, LEDs, TVC, and pyro channels.");
    rocket.printMessage(HELP_STR);
}

void loop() {

    recvOneChar();
#if USE_BLE
    if (rocket.bleOn) rocket.updateBle();
#endif
    rocket.updateTvc();
    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();
    rocket.updatePyros();
    rocket.updateBuzzer();
    rocket.updateAngleLeds();
    rocket.updateDataLog();

    if (newCommand == true) {
        switch (receivedChar) {
            case 'U':
                rocket.printMessage("Unlocking TVC");
                rocket.tvc.unlock();
                break;
            case 'L':
                rocket.printMessage("Locking TVC");
                rocket.tvc.lock();
                break;
            case 'K':
                rocket.printMessage("Toggling LEDs");
                rocket.toggleLeds();
                break;
            case 'C':
                rocket.calibrateAndLog();
                break;
            case 'O':
                rocket.printMessage("Orientation: ");
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
                rocket.toggleDataLog();
                break;
            case 'P':
                rocket.printMessage("Time per loop: ", false);
                rocket.printMessage(rocket.deltaTime);

                rocket.printMessage("Loop rate: ", false);
                rocket.printMessage(1 / rocket.deltaTime);
                break;

            case 'Y':
                rocket.printMessage("Running test routine");
                rocket.logMessage("Running test routine");
                rocket.tvc.testRoutine();
                rocket.printMessage("Test routine complete!");
                rocket.logMessage("Test routine complete!");
                break;

            case 'E':
                experimentMode = !experimentMode;
                if (experimentMode) {
                    rocket.setState(76);
                    rocket.printMessage("Experiment Mode ON");
                    rocket.logMessage("Experiment Mode ON - Running Test");
                    rocket.initAngles();
                    rocket.calibrateAndLog();
                    rocket.disableCompl();
                    rocket.printMessage("Unlocking TVC");
                    rocket.logMessage("Unlocking TVC");
                    rocket.tvc.unlock();

                    rocket.printMessage("Firing Engine");
                    rocket.logMessage("Firing Engine");
                    rocket.firePyro1();

                    rocket.printMessage("Releasing from stand");
                    rocket.logMessage("Releasing from stand");
                    rocket.firePyro2();
                    currentMs = millis();
                    rocket.setDataLog(true);
                }
                else {
                    rocket.setState(0);
                    rocket.printMessage("Experiment Mode OFF - Test Complete");
                    rocket.logMessage("Experiment Mode OFF - Test Complete");
                    rocket.printMessage("Locking TVC");
                    rocket.logMessage("Locking TVC");
                    rocket.tvc.lock();

                    rocket.printMessage("Disarming Pyros");
                    rocket.logMessage("Disarming Pyros");
                    rocket.pyro1_motor.disarm();
                    rocket.pyro2_land.disarm();
                }
                break;

            default:
                rocket.printMessage("Invalid command");
                break;
        }
        newCommand = false;
    }

    if (((millis() - currentMs) > 7000) && (experimentMode == true)) {
        experimentMode = false;
        rocket.setState(42);
        rocket.printMessage("Experiment Mode OFF - Test Complete");
        rocket.logMessage("Experiment Mode OFF - Test Complete");
        rocket.printMessage("Locking TVC");
        rocket.logMessage("Locking TVC");
        rocket.tvc.lock();

        rocket.printMessage("Disarming Pyros");
        rocket.logMessage("Disarming Pyros");
        rocket.pyro1_motor.disarm();
        rocket.pyro2_land.disarm();

        currentMs = millis();
        rocket.disableCompl();
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
        float yaw = dir.z;
        float pitch = dir.y;
        float roll = dir.x;
        rocket.printMessage(yaw, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(pitch, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(roll);
    }

    rocket.updateTime();
}

