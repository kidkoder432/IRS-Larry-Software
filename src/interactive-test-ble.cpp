// Test all components and features

#include <Arduino.h>
#include <Serial.h>
#include <Servo.h>

#include <orientation.h>

#include <tvc.h>
#include <leds.h>
#include <pyro.h>

#include <alt.h>
#include <Arduino_LPS22HB.h>

#include <config.h>
#include <datalog.h>
#include <prints.h>
#include <HardwareBLESerial.h>

HardwareBLESerial& bleSerial = HardwareBLESerial::getInstance();
bool bleOn = false;

double yaw, pitch, roll;
TVC tvc;
double x_out, y_out;
SensorReadings readings;
Vec3D dir;
Biases biases;
Kalman kx, ky;
Quaternion attitude;
Config config;
long long lastMicros;

double vertVel = 0;

double ALPHA = 0.05;

char receivedChar;
bool newCommand = false;

bool dirOutLock = true;
bool sensorOutLock = true;

bool experimentMode = false;

bool logData = true;
bool led = true;
File dataFile;
File logFile;

PyroChannel pyro1_motor(PYRO_LANDING_MOTOR_IGNITION, 1000);
PyroChannel pyro2_land(PYRO_LANDING_LEGS_DEPLOY, 1000);

double pressureOffset = 0;

int currentState = 42;   // Test state = 42

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

    // Init serial and sensors
    Serial.begin(115200);

    initIMU();
    BARO.begin();

    // Init angles
    readSensors(readings, biases);
    yaw = atan2(readings.ax, -sign(readings.ay) * sqrt(readings.az * readings.az + readings.ay * readings.ay)) * 180 / PI;
    pitch = atan2(readings.az, readings.ay) * 180 / PI;

    dir.x = yaw;
    dir.y = pitch;
    dir.z = 0;

    attitude = Quaternion();

    // Init pyros
    pyro1_motor.begin();
    pyro2_land.begin();

    // Init LEDs
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    // Init SD Card | Read Config
    SD.begin(10);
    logFile = SD.open("log.txt", O_TRUNC | O_WRITE);

    if (!logFile) {
        msgPrintln(bleOn, bleSerial, "Failed to open log file!");
        while (1) {
            flash(COLOR_RED);

        }
    }

    logStatus("Initialized", logFile);
    config = readConfig();
    logStatus("Config read successfully", logFile);

    logStatus("Calibrating Barometer", logFile);
    pressureOffset = calculateOffset(config.PRESSURE_0);
    logStatus("Barometer calibrated", logFile);


    // Open data file
    dataFile = SD.open("data.csv", O_TRUNC | O_WRITE);

    if (!dataFile) {
        msgPrintln(bleOn, bleSerial, "Failed to open data file!");
        while (1) {
            flash(COLOR_RED);

        }
    }
    dataFile.println("Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Xout,Yout,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");

    // Init TVC
    tvc.begin();
    tvc.lock();
    tvc.updatePID(tvc.pid_x, config.Kp, config.Ki, config.Kd, config.N);
    tvc.updatePID(tvc.pid_y, config.Kp, config.Ki, config.Kd, config.N);

    if (config.FILTER_KALMAN == 1) {
        msgPrintln(bleOn, bleSerial, "Initializing Kalman filter...");
        logStatus("Initializing Kalman filter", logFile);
        kx.setAngle(yaw);
        ky.setAngle(pitch);
    }
    else {
        msgPrintln(bleOn, bleSerial, "Using complementary filter");
        logStatus("Using complementary filter", logFile);
    }

    delay(1000);

    // Init BLE
    msgPrintln(bleOn, bleSerial, "Initializing BLE...");
    logStatus("Initializing BLE", logFile);

    if (!bleSerial.beginAndSetupBLE("Tax Collector")) {
        msgPrintln(bleOn, bleSerial, "Failed to initialize BLE!");
        logStatus("ERR: Failed to initialize BLE! ", logFile);
        logFile.close();
        dataFile.close();
        while (1) {
            flash(COLOR_PURPLE);

        }
    }

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
        logStatus("HardwareBLESerial central device connected!", logFile);
    }
    else {
        bleOn = false;
        msgPrintln(bleOn, bleSerial, "BLE not connected! Using USB serial...");
        logStatus("WARN: BLE not connected! Using USB serial...", logFile);

    }

    // Calibrate Gyro
    msgPrintln(bleOn, bleSerial, "Calibrating Sensors...");
    logStatus("Calibrating Sensors", logFile);
    biases = calibrateSensors();

    char buf[64];
    snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);


    char calStr[128];
    strcat(calStr, "Calibration values: ");
    strcat(calStr, buf);
    logStatus("Calibration complete", logFile);
    logStatus(calStr, logFile);

    msgPrintln(bleOn, bleSerial, "Initialized");
    msgPrintln(bleOn, bleSerial, R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");

    delay(200);
    lastMicros = micros();

    pyro1_motor.arm();
    pyro2_land.arm();
}

void loop() {

    bleSerial.poll();

    recvOneChar();

    readSensors(readings, biases);

    vertVel -= readings.ay * 9.80665 * DELTA_TIME;

    Vec2D tvc_out = tvc.update(dir, DELTA_TIME);
    x_out = tvc_out.x;
    y_out = tvc_out.y;

    dir = get_angles_quat(readings, attitude, DELTA_TIME);

    roll = dir.x;
    pitch = dir.y;
    yaw = dir.z;

    pyro1_motor.update();
    pyro2_land.update();

    if (yaw > 180) {
        yaw = yaw - 360;
    }
    else if (yaw < -180) {
        yaw = yaw + 360;
    }

    if (pitch > 180) {
        pitch = pitch - 360;
    }

    else if (pitch < -180) {
        pitch = pitch + 360;
    }

    if (newCommand == true) {
        switch (receivedChar) {
            case 'U':
                msgPrintln(bleOn, bleSerial, "Unlocking TVC");
                logStatus("Unlocking TVC", logFile);
                tvc.pid_x.reset();
                tvc.pid_y.reset();
                tvc.unlock();
                break;
            case 'L':
                msgPrintln(bleOn, bleSerial, "Locking TVC");
                logStatus("Locking TVC", logFile);
                tvc.lock();
                break;
            case 'K':
                msgPrintln(bleOn, bleSerial, "Toggling LEDs");
                logStatus("Toggling LEDs", logFile);
                led = !led;
                break;
            case 'C':
                msgPrintln(bleOn, bleSerial, "Calibrating Sensors...");
                logStatus("Calibrating Sensors", logFile);
                biases = calibrateSensors();

                char buf[64];
                snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);


                char calStr[128];
                strcat(calStr, "Calibration values: ");
                strcat(calStr, buf);
                logStatus("Calibration complete", logFile);
                logStatus(calStr, logFile);
                break;
            case 'O':
                msgPrint(bleOn, bleSerial, "Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                msgPrint(bleOn, bleSerial, "Altitude: ");
                msgPrintln(bleOn, bleSerial, getAltitude(config.PRESSURE_0, pressureOffset));
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
                logStatus("Pyro 1: Motor Ignition", logFile);
                pyro1_motor.fire();
                // fire_pyro_test(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                msgPrintln(bleOn, bleSerial, "Pyro 2: Landing Legs Deploy");
                logStatus("Pyro 2: Landing Legs Deploy", logFile);
                pyro2_land.fire();
                // fire_pyro_test(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                msgPrintln(bleOn, bleSerial, "Reading SD Card Info");
                logStatus("Reading SD Card Info", logFile);
                sdCardInfo();
                break;
            case 'Q':
                msgPrintln(bleOn, bleSerial, "Resetting Angles");
                logStatus("Resetting Angles", logFile);
                dir = Vec3D(0, 0, 0);
                attitude = Quaternion();
                break;
            case 'H':
                msgPrintln(bleOn, bleSerial, HELP_STR);
                break;

            case 'D':
                logData = !logData;
                if (logData) {
                    SD.begin(10);
                    msgPrintln(bleOn, bleSerial, "Logging Data ON");
                    logStatus("Logging Data ON", logFile);
                }
                else {
                    msgPrintln(bleOn, bleSerial, "Logging Data OFF");
                    logStatus("Logging Data OFF", logFile);
                }
                break;
            case 'P':
                msgPrint(bleOn, bleSerial, "Px: ");
                msgPrint(bleOn, bleSerial, tvc.pid_x.p);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Ix: ");
                msgPrint(bleOn, bleSerial, tvc.pid_x.i);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Dx: ");
                msgPrintln(bleOn, bleSerial, tvc.pid_x.d);

                msgPrint(bleOn, bleSerial, "Py: ");
                msgPrint(bleOn, bleSerial, tvc.pid_y.p);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Iy: ");
                msgPrint(bleOn, bleSerial, tvc.pid_y.i);
                msgPrint(bleOn, bleSerial, "; ");
                msgPrint(bleOn, bleSerial, "Dy: ");
                msgPrintln(bleOn, bleSerial, tvc.pid_y.d);
                break;

            case 'E':
                experimentMode = !experimentMode;
                if (experimentMode) {
                    currentState = 127;
                    msgPrintln(bleOn, bleSerial, "Experiment Mode ON");
                    tvc.unlock();
                    logData = true;
                }
                else {
                    currentState = 42;
                    msgPrintln(bleOn, bleSerial, "Experiment Mode OFF");
                    tvc.lock();
                    logData = false;
                }
                break;

            default:
                msgPrintln(bleOn, bleSerial, "Invalid command");
                break;
        }
        newCommand = false;
    }

    if (led) {
        if (abs(yaw) >= 15) {
            showColor(COLOR_RED);
            if (abs(pitch) >= 15) {
                showColor(COLOR_PURPLE);
            }
        }
        else {
            showColor(COLOR_GREEN);
            if (abs(pitch) >= 15) {
                showColor(COLOR_LIGHTBLUE);
            }
        }
    }
    else {
        showColor(COLOR_OFF);
    }

    if (!sensorOutLock) {
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
        msgPrint(bleOn, bleSerial, yaw);
        msgPrint(bleOn, bleSerial, " ");
        msgPrint(bleOn, bleSerial, pitch);
        msgPrint(bleOn, bleSerial, " ");
        msgPrintln(bleOn, bleSerial, roll);
    }

    if (logData) {

        DataPoint p;
        p.timestamp = lastMicros;
        p.DELTA_T = DELTA_TIME;
        p.r = readings;
        p.o = Vec3D(roll, pitch, yaw);
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = getAltitude(config.PRESSURE_0, pressureOffset);
        p.currentState = currentState;
        p.vert_vel = vertVel;
        p.px = tvc.pid_x.p;
        p.ix = tvc.pid_x.i;
        p.dx = tvc.pid_x.d;
        p.py = tvc.pid_y.p;
        p.iy = tvc.pid_y.i;
        p.dy = tvc.pid_y.d;

        logDataPoint(p, dataFile);

    }


    DELTA_TIME = (micros() - lastMicros) / 1000000.0;
    lastMicros = micros();
}
