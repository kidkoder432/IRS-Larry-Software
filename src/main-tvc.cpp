// Main flight loop

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
Quaternion attitude;

Config config;

long long lastLoopTime;

double vertVel = 0;

bool logData = true;
File dataFile;
File logFile;

double pressureOffset = 0;

int currentState = 0;   // 0: Pad-Idle
bool aborted = false;

double mag3(double a, double b, double c) {
    return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
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

    dir.x = 0;
    dir.y = pitch;
    dir.z = yaw;

    attitude = Quaternion();

    // Init pyros
    pinMode(PYRO_LANDING_LEGS_DEPLOY, OUTPUT);
    pinMode(PYRO_LANDING_MOTOR_IGNITION, OUTPUT);

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

    delay(1000);

    // Init BLE
    msgPrintln(bleOn, bleSerial, "Initializing BLE...");
    logStatus("Initializing BLE", logFile);

    if (!bleSerial.beginAndSetupBLE("Tax Collector")) {
        msgPrintln(bleOn, bleSerial, "Failed to initialize BLE!");
        logStatus("WARN: Failed to initialize BLE! ", logFile);
        logFile.close();
        dataFile.close();
        while (1) {
            flash(COLOR_PURPLE);

        }
    }

    int waits = 0;
    while (!bleSerial && waits < 30) {

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

    delay(200);
    lastLoopTime = micros();

}

void loop() {

    bleSerial.poll();
    readSensors(readings, biases);

    vertVel -= readings.ay * 9.80665 * DELTA_TIME;

    Vec2D tvc_out = tvc.update(dir, DELTA_TIME);
    x_out = tvc_out.x;
    y_out = tvc_out.y;

    dir = get_angles_quat(readings, attitude, DELTA_TIME);

    roll = dir.x;
    pitch = dir.y;
    yaw = dir.z;

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

    LED(currentState);

    // --- States --- //
    switch (currentState) {
        case 0:  // Pad-Idle
            tvc.lock();
            break;
        case 1:  // Powered Ascent
            tvc.unlock();
            break;
        case 3:  // Coast
            tvc.lock();
            break;
        case 4:  // Powered Descent
            tvc.unlock();
            break;
        case 5:  // Touchdown
            break;

        case 127:  // Abort
            logData = false;
            bleOn = false;
            bleSerial.end();
            tvc.abort();
            dataFile.close();
            logFile.close();

            break;
    }

    // --- State Transitions --- //

    // ABORT: State -> 127
    if (abs(yaw) >= 45 || abs(pitch) >= 45) {
        logStatus("ERR: ABORT - UNSTABLE", logFile);
        currentState = 127;
    }

    // TOUCHDOWN: State 3 -> 5
    if (currentState == 3 && mag3(readings.ax, readings.ay, readings.az) <= 1.5) {
        currentState = 5;
    }

    // STAGE 1 BURNOUT: State 1 -> 3
    if (currentState == 1 && mag3(readings.ax, readings.ay, readings.az) <= 1.1) {
        currentState = 3;
    }

    // LAUNCH: State 0 -> 1
    if (currentState == 0 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
        currentState = 1;
    }

    // --- Data Logging --- //
    vertVel -= readings.ay * 9.81 * DELTA_TIME;

    if (logData) {

        DataPoint p;
        p.timestamp = lastLoopTime;
        p.DELTA_T = DELTA_TIME;
        p.r = readings;
        p.o = dir;
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


    DELTA_TIME = (micros() - lastLoopTime) / 1000000.0;
    lastLoopTime = micros();
}
