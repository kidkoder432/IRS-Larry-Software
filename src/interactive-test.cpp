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
bool newData = false;

bool dirOutLock = true;
bool sensorOutLock = true;

bool logData = true;
bool led = true;
File dataFile;
File logFile;
char filename[64] = "data.csv";

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
H: Help)";

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

    tvc.begin();
    tvc.lock();

    SD.begin(10);

    logFile = SD.open("log.txt", FILE_WRITE);
    logStatus("Initialized", logFile);

    Serial.println("Reading config...");
    config = readConfig();
    Serial.println("Config read successfully");
    Serial.print("Kp: ");
    Serial.println(config.Kp);
    Serial.print("Ki: ");
    Serial.println(config.Ki);
    Serial.print("Kd: ");
    Serial.println(config.Kd);
    Serial.print("N: ");
    Serial.println(config.N);
    Serial.print("FILTER_KALMAN: ");
    Serial.println(config.FILTER_KALMAN);
    Serial.print("Pressure Reference: ");
    Serial.println(config.PRESSURE_0);
    logStatus("Config read successfully", logFile);

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

    tvc.updatePID(tvc.pid_x, config.Kp, config.Ki, config.Kd, config.N);
    tvc.updatePID(tvc.pid_y, config.Kp, config.Ki, config.Kd, config.N);

    delay(1000);

    Serial.println("Initialized");
    Serial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");

    // Serial.println(HELP_STR);

    dataFile = SD.open(filename, FILE_WRITE);
    delay(200);
    Serial.println(dataFile);

}


void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        Serial.println(receivedChar);
        newData = true;
    }
}

void loop() {
    recvOneChar();

    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);

    vertVel -= readings.ay * 9.80665 * DELTA_TIME;

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

    if (newData == true) {
        switch (receivedChar) {
            case 'U':
                Serial.println("Unlocking TVC");
                logStatus("Unlocking TVC", logFile);
                tvc.unlock();
                break;
            case 'L':
                Serial.println("Locking TVC");
                logStatus("Locking TVC", logFile);
                tvc.lock();
                break;
            case 'K':
                Serial.println("Toggling LEDs");
                logStatus("Toggling LEDs", logFile);
                led = !led;
                break;
            case 'C':
                Serial.println("Calibrating Gyro");
                logStatus("Calibrating Gyro", logFile);
                calibrateGyro();
                break;
            case 'O':
                Serial.print("Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                Serial.print("Altitude: ");
                Serial.println(getAltitude(config.PRESSURE_0));
                break;
            case 'R':
                Serial.println("Sensor Readings: ");
                dirOutLock = true;
                sensorOutLock = false;
                break;
            case 'X':
                sensorOutLock = true;
                dirOutLock = true;
                break;
            case 'G':
                Serial.println("Pyro 1: Motor Ignition");
                logStatus("Pyro 1: Motor Ignition", logFile);
                fire_pyro_test(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                Serial.println("Pyro 2: Landing Legs Deploy");
                logStatus("Pyro 2: Landing Legs Deploy", logFile);
                fire_pyro_test(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                Serial.println("Reading SD Card Info");
                logStatus("Reading SD Card Info", logFile);
                sdCardInfo();
                break;
            case 'Q':
                Serial.println("Resetting Angles");
                logStatus("Resetting Angles", logFile);
                yaw = 0;
                pitch = 0;
                break;
            case 'H':
                Serial.println(HELP_STR);
                break;

            case 'D':
                logData = !logData;
                if (logData) {
                    SD.begin(10);
                    Serial.println("Logging Data ON");
                    logStatus("Logging Data ON", logFile);
                }
                else {
                    Serial.println("Logging Data OFF");
                    logStatus("Logging Data OFF", logFile);
                }
                break;
            default:
                Serial.println("Invalid command");
                break;
        }
        newData = false;
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
        Serial.print(readings.ax);
        Serial.print(" ");
        Serial.print(readings.ay);
        Serial.print(" ");
        Serial.println(readings.az);

        Serial.print("Gyroscope: ");
        Serial.print(readings.gx);
        Serial.print(" ");
        Serial.print(readings.gy);
        Serial.print(" ");
        Serial.println(readings.gz);
    }

    if (!dirOutLock) {
        Serial.print(yaw);
        Serial.print(" ");
        Serial.println(pitch);
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
        p.kp = tvc.pid_x.Kp;
        p.ki = tvc.pid_x.Ki;
        p.kd = tvc.pid_x.Kd;

        logDataPoint(p, dataFile);

        if (millis() % 100 == 0) {
            dataFile.flush();
        }
    }


    DELTA_TIME = (micros() - lastMicros) / 1000000.0;
    lastMicros = micros();
}