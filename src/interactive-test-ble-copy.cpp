// Test all components and features

#include <routines.h>

#define SD_ATTACHED 1

Rocket rocket;

HardwareBLESerial& bleSerial = rocket.getBle();
bool bleOn = true;

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
P: Print PID Values
B: Edit Config
E: Toggle Experiment/Test Mode
Z: Switch Bluetooth to USB (override)
H: Help)";

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        msgPrintln(bleOn, bleSerial, receivedChar);
        newCommand = true;
    }
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        msgPrintln(bleOn, bleSerial, receivedChar);
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
    rocket.getSdInfo();
    msgPrintln(bleOn, bleSerial, "SD card initialized!");
    rocket.initLogs();
    rocket.initConfig();
    msgPrintln(bleOn, bleSerial, "Config read successfully!");

    rocket.initBle();

    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 and !override) {
        flash(COLOR_BLUE);
        recvOneChar();
        if (newCommand) {
            switch (receivedChar) {
                case 'Z':
                    bleOn = false;
                    override = true;
                    break;
                default:
                    override = false;
                    break;
            }
        }

        if (millis() % 1000 < 40) {
            msgPrintln(bleOn, bleSerial, "BLE not connected, waiting...");
            waits++;
            delay(50);
            showColor(COLOR_OFF);

        }
    }

    if (bleSerial) {
        bleOn = true;
        msgPrintln(bleOn, bleSerial, "HardwareBLESerial central device connected!");
        rocket.logMessage("HardwareBLESerial central device connected!");
    }
    else {
        bleOn = false;
        msgPrintln(bleOn, bleSerial, "BLE not connected! Using USB serial...");
        rocket.logMessage("WARN: BLE not connected! Using USB serial...");

    }
#endif

    // Init sensors and angles
    rocket.setupSensors();
    rocket.calibrateAndLog();
    rocket.initAngles();

    // init hardware
    rocket.initLeds();
    rocket.initTvc();

    // init pyros and complete only after other inits succeed
    rocket.initPyros();
    rocket.finishSetup();
}

void loop() {

    bleSerial.poll();

    recvOneChar();

    rocket.updateBle();
    rocket.updateTvc();
    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();
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
                rocket.ledsOn = !rocket.ledsOn;
                break;
            case 'C':
                rocket.calibrateAndLog();
                break;
            case 'O':
                msgPrint(bleOn, bleSerial, "Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                msgPrint(bleOn, bleSerial, "Altitude: ");
                msgPrintln(bleOn, bleSerial, rocket.getAlt());
                break;
            case 'R':
                msgPrintln(bleOn, bleSerial, "Sensor Readings: ");
                dirOutLock = true;
                sensorOutLock = false;
                break;
            case 'X':
                sensorOutLock = true;
                dirOutLock = true;
                break;
            case 'G':
                msgPrintln(bleOn, bleSerial, "Pyro 1: Motor Ignition");
                rocket.firePyro1();
                // fire_pyro_test(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                msgPrintln(bleOn, bleSerial, "Pyro 2: Landing Legs Deploy");
                rocket.firePyro2();
                // fire_pyro_test(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                msgPrintln(bleOn, bleSerial, "Reading SD Card Info");
                rocket.getSdInfo();
                break;
            case 'Q':

                rocket.resetAngles();
                break;
            case 'H':
                msgPrintln(bleOn, bleSerial, HELP_STR);
                break;

            case 'D':
                rocket.doLog = !rocket.doLog;
                break;
            case 'P':
                msgPrint(bleOn, bleSerial, "Time Step: ");
                msgPrintln(bleOn, bleSerial, rocket.deltaTime);
                msgPrint(bleOn, bleSerial, "Loop rate: ");
                msgPrintln(bleOn, bleSerial, 1 / rocket.deltaTime);
                msgPrint(bleOn, bleSerial, "Px: ");
                msgPrint(bleOn, bleSerial, rocket.tvc.pid_x.p);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Ix: ");
                msgPrint(bleOn, bleSerial, rocket.tvc.pid_x.i);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Dx: ");
                msgPrintln(bleOn, bleSerial, rocket.tvc.pid_x.d);

                // msgPrint(bleOn, bleSerial, "Py: ");
                // msgPrint(bleOn, bleSerial, tvc.pid_y.p);
                // msgPrint(bleOn, bleSerial, "; ");
                // msgPrint(bleOn, bleSerial, "Iy: ");
                // msgPrint(bleOn, bleSerial, tvc.pid_y.i);
                // msgPrint(bleOn, bleSerial, "; ");
                // msgPrint(bleOn, bleSerial, "Dy: ");
                // msgPrintln(bleOn, bleSerial, tvc.pid_y.d);
                break;

            case 'E':
                experimentMode = !experimentMode;
                if (experimentMode) {
                    rocket.currentState = 127;
                    msgPrintln(bleOn, bleSerial, "Experiment Mode ON");
                    rocket.tvc.unlock();
                    if (!rocket.doLog) rocket.setDataLog(true);
                }
                else {
                    rocket.currentState = 42;
                    msgPrintln(bleOn, bleSerial, "Experiment Mode OFF");
                    rocket.tvc.lock();
                }
                break;

            default:
                msgPrintln(bleOn, bleSerial, "Invalid command");
                break;
        }
        newCommand = false;
    }

    if (!sensorOutLock) {
        SensorReadings readings = rocket.getReadings();
        msgPrint(bleOn, bleSerial, "Accelerometer: ");
        msgPrint(bleOn, bleSerial, readings.ax);
        msgPrint(bleOn, bleSerial, " ");
        msgPrint(bleOn, bleSerial, readings.ay);
        msgPrint(bleOn, bleSerial, " ");
        msgPrintln(bleOn, bleSerial, readings.az);

        msgPrint(bleOn, bleSerial, "Gyroscope: ");
        msgPrint(bleOn, bleSerial, readings.gx);
        msgPrint(bleOn, bleSerial, " ");
        msgPrint(bleOn, bleSerial, readings.gy);
        msgPrint(bleOn, bleSerial, " ");
        msgPrintln(bleOn, bleSerial, readings.gz);
    }

    if (!dirOutLock) {
        // msgPrint(bleOn, bleSerial, attitude.a);
        // msgPrint(bleOn, bleSerial, " ");
        // msgPrint(bleOn, bleSerial, attitude.b);
        // msgPrint(bleOn, bleSerial, " ");
        // msgPrint(bleOn, bleSerial, attitude.c);
        // msgPrint(bleOn, bleSerial, " ");
        // msgPrint(bleOn, bleSerial, attitude.d);
        // msgPrint(bleOn, bleSerial, " ");
        Vec3D dir = rocket.getDir();
        double yaw = dir.x;
        double pitch = dir.y;
        double roll = dir.z;
        msgPrint(bleOn, bleSerial, yaw);
        msgPrint(bleOn, bleSerial, " ");
        msgPrint(bleOn, bleSerial, pitch);
        msgPrint(bleOn, bleSerial, " ");
        msgPrintln(bleOn, bleSerial, roll);
    }

    rocket.updateTime();
}

