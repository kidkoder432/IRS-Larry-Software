// Test all components and features

#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>
#include <datalog.h>
#include <pyro.h>
#include <alt.h>
#include <Servo.h>
#include <Arduino_LPS22HB.h>
#include <tvc.h>
#include <config.h>

#include <HardwareBLESerial.h>

HardwareBLESerial& bleSerial = HardwareBLESerial::getInstance();

double yaw, pitch;
TVC tvc;
double x_out, y_out;
SensorReadings readings;
Vec2D dir;
Biases biases;
Kalman kx, ky;
Config config;
long long lastMicros = micros();

float vertVel = 0;

double ALPHA = 0.05;

char receivedChar;
bool newCommand = false;

bool dirOutLock = true;
bool sensorOutLock = true;

bool logData = true;
bool led = true;
File dataFile;
File logFile;

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
Z: Switch Bluetooth to USB (override)
H: Help)";

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        Serial.println(receivedChar);
        newCommand = true;
    }
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        Serial.println(receivedChar);
        newCommand = true;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void setup() {

    // Init serial and sensors
    Serial.begin(9600);
    IMU.begin();
    BARO.begin();

    // Init angles
    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, ay) * 180 / PI;

    // Init pyros
    pinMode(PYRO_LANDING_LEGS_DEPLOY, OUTPUT);
    pinMode(PYRO_LANDING_MOTOR_IGNITION, OUTPUT);

    // Init LEDs
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    // Init SD Card | Read Config
    SD.begin(10);
    logFile = SD.open("log.txt", O_WRITE);

    if (!logFile) {
        Serial.println("Failed to open log file!");

        while (1) {}
    }

    logStatus("Initialized", logFile);
    config = readConfig();
    logStatus("Config read successfully", logFile);

    // Open data file
    dataFile = SD.open("data.csv", O_WRITE);

    if (!dataFile) {
        Serial.println("Failed to open data file!");
        while (1) {}
    }
    dataFile.println("Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Xout,Yout,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");

    // Init TVC
    tvc.begin();
    tvc.lock();
    tvc.updatePID(tvc.pid_x, config.Kp, config.Ki, config.Kd, config.N);
    tvc.updatePID(tvc.pid_y, config.Kp, config.Ki, config.Kd, config.N);

    if (config.FILTER_KALMAN == 1) {
        Serial.println("Initializing Kalman filter...");
        logStatus("Initializing Kalman filter", logFile);
        kx.setAngle(yaw);
        ky.setAngle(pitch);
    }
    else {
        Serial.println("Using complementary filter");
        logStatus("Using complementary filter", logFile);
    }



    delay(1000);

    // Init BLE
    Serial.println("Initializing BLE...");
    logStatus("Initializing BLE", logFile);

    if (!bleSerial.beginAndSetupBLE("Tax Collector")) {
        Serial.println("Failed to initialize BLE!");
        logStatus("ERR: Failed to initialize BLE! ", logFile);
        logFile.close();
        dataFile.close();
        while (1) {}
    }

    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 and !override) {
        
        recvOneChar();
        if (newCommand) {
            switch (receivedChar) {
                case 'Z':
                    override = true;
                    break;
                default:
                    override = false;
                    break;
            }
        }
        
        if (millis() % 1000 == 0) {
            Serial.println("BLE not connected, waiting...");
            waits++;
        }
        delay(20);
    }

    if (bleSerial) {
        Serial.println("HardwareBLESerial central device connected!");
        logStatus("HardwareBLESerial central device connected!", logFile);
    }
    else {
        Serial.println("BLE not connected! Using USB serial...");
        logStatus("WARN: BLE not connected! Using USB serial...", logFile);

    }

    // Calibrate Gyro
    Serial.println("Calibrating Gyro...");
    logStatus("Calibrating Gyro", logFile);
    biases = calibrateGyro();

    char buf[64];
    snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);

    logStatus("Calibration complete", logFile);
    logStatus(strcat(buf, "Calibration values: "), logFile);

    Serial.println("Initialized");
    bleSerial.println("Initialized");
    Serial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");
    bleSerial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");
    delay(200);

}

void loop() {

    bleSerial.poll();

    recvOneChar();

    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);

    vertVel -= readings.ax * 9.80665 * DELTA_TIME;

    Vec2D tvc_out = tvc.update(dir, DELTA_TIME);
    x_out = tvc_out.x;
    y_out = tvc_out.y;

    if (config.FILTER_KALMAN) {
        dir = get_angles_kalman(DELTA_TIME, readings, kx, ky, biases);

    }

    else {
        dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, yaw, pitch, biases);

    }
    yaw = dir.x;
    pitch = dir.y;

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
                Serial.println("Unlocking TVC");
                bleSerial.println("Unlocking TVC");
                logStatus("Unlocking TVC", logFile);
                tvc.pid_x.reset();
                tvc.pid_y.reset();
                tvc.unlock();
                break;
            case 'L':
                Serial.println("Locking TVC");
                bleSerial.println("Locking TVC");
                logStatus("Locking TVC", logFile);
                tvc.lock();
                break;
            case 'K':
                Serial.println("Toggling LEDs");
                bleSerial.println("Toggling LEDs");
                logStatus("Toggling LEDs", logFile);
                led = !led;
                break;
            case 'C':
                Serial.println("Calibrating Gyro");
                bleSerial.println("Calibrating Gyro");
                logStatus("Calibrating Gyro", logFile);
                calibrateGyro();
                break;
            case 'O':
                Serial.print("Orientation: ");
                bleSerial.print("Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                Serial.print("Altitude: ");
                bleSerial.print("Altitude: ");
                Serial.println(getAltitude(config.PRESSURE_0));
                bleSerial.println(getAltitude(config.PRESSURE_0));
                break;
            case 'R':
                Serial.println("Sensor Readings: ");
                bleSerial.println("Sensor Readings: ");
                dirOutLock = true;
                sensorOutLock = false;
                break;
            case 'X':
                sensorOutLock = true;
                dirOutLock = true;
                break;
            case 'G':
                Serial.println("Pyro 1: Motor Ignition");
                bleSerial.println("Pyro 1: Motor Ignition");
                logStatus("Pyro 1: Motor Ignition", logFile);
                fire_pyro_test(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                Serial.println("Pyro 2: Landing Legs Deploy");
                bleSerial.println("Pyro 2: Landing Legs Deploy");
                logStatus("Pyro 2: Landing Legs Deploy", logFile);
                fire_pyro_test(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                Serial.println("Reading SD Card Info");
                bleSerial.println("Reading SD Card Info");
                logStatus("Reading SD Card Info", logFile);
                sdCardInfo();
                break;
            case 'Q':
                Serial.println("Resetting Angles");
                bleSerial.println("Resetting Angles");
                logStatus("Resetting Angles", logFile);
                yaw = 0;
                pitch = 0;
                break;
            case 'H':
                Serial.println(HELP_STR);
                bleSerial.println(HELP_STR);
                break;

            case 'D':
                logData = !logData;
                if (logData) {
                    SD.begin(10);
                    Serial.println("Logging Data ON");
                    bleSerial.println("Logging Data ON");
                    logStatus("Logging Data ON", logFile);
                }
                else {
                    Serial.println("Logging Data OFF");
                    bleSerial.println("Logging Data OFF");
                    logStatus("Logging Data OFF", logFile);
                }
                break;
            case 'P':
                Serial.print("Px: ");
                bleSerial.print("Px: ");
                Serial.print(tvc.pid_x.p);
                bleSerial.print(tvc.pid_x.p);
                Serial.print("; ");
                bleSerial.print("; ");
                Serial.print("Ix: ");
                bleSerial.print("Ix: ");
                Serial.print(tvc.pid_x.i);
                bleSerial.print(tvc.pid_x.i);
                Serial.print("; ");
                bleSerial.print("; ");
                Serial.print("Dx: ");
                bleSerial.print("Dx: ");
                Serial.println(tvc.pid_x.d);
                bleSerial.println(tvc.pid_x.d);

                Serial.print("Py: ");
                bleSerial.print("Py: ");
                Serial.print(tvc.pid_y.p);
                bleSerial.print(tvc.pid_y.p);
                Serial.print("; ");
                bleSerial.print("; ");
                Serial.print("Iy: ");
                bleSerial.print("Iy: ");
                Serial.print(tvc.pid_y.i);
                bleSerial.print(tvc.pid_y.i);
                Serial.print("; ");
                bleSerial.print("; ");
                Serial.print("Dy: ");
                bleSerial.print("Dy: ");
                Serial.println(tvc.pid_y.d);
                bleSerial.println(tvc.pid_y.d);
                break;
            default:
                Serial.println("Invalid command");
                bleSerial.println("Invalid command");
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
        Serial.print("Accelerometer: ");
        bleSerial.print("Accelerometer: ");
        Serial.print(readings.ax);
        bleSerial.print(readings.ax);
        Serial.print(" ");
        bleSerial.print(" ");
        Serial.print(readings.ay);
        bleSerial.print(readings.ay);
        Serial.print(" ");
        bleSerial.print(" ");
        Serial.println(readings.az);
        bleSerial.println(readings.az);

        Serial.print("Gyroscope: ");
        bleSerial.print("Gyroscope: ");
        Serial.print(readings.gx);
        bleSerial.print(readings.gx);
        Serial.print(" ");
        bleSerial.print(" ");
        Serial.print(readings.gy);
        bleSerial.print(readings.gy);
        Serial.print(" ");
        bleSerial.print(" ");
        Serial.println(readings.gz);
        bleSerial.println(readings.gz);
    }

    if (!dirOutLock) {
        Serial.print(yaw);
        bleSerial.print(yaw);
        Serial.print(" ");
        bleSerial.print(" ");
        Serial.println(pitch);
        bleSerial.println(pitch);
    }

    if (logData) {

        DataPoint p;
        p.timestamp = micros();
        p.r = readings;
        p.o = Vec2D(yaw, pitch);
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = getAltitude(config.PRESSURE_0);
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
