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

double yaw, pitch;
TVC tvc;
SensorReadings readings;
Orientation dir;
Biases biases;
long long lastMicros = micros();

double ALPHA = 0.05;

char receivedChar;
bool newData = false;

bool dirOutLock = true;
bool sensorOutLock = true;

const char HELP_STR[] = 
R"(Input commands to test the following:
U: Unlock the TVC
L: Lock the TVC
C: Calibrate the Gyro
O: Print Orientation
A: Print Altitude
R: Print Sensor Readings
X: Cancel Printing
G: Activate Pyro 1 (Motor Ignition)
T: Activate Pyro 2 (Landing Legs Deploy)
S: Read SD Card
Q: Reset Angles
H: Help)";

void setup() {
    Serial.begin(9600);
    IMU.begin();
    BARO.begin();
    pinMode(LED_BUILTIN, OUTPUT);

    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(PYRO_LANDING_LEGS_DEPLOY, OUTPUT);
    pinMode(PYRO_LANDING_MOTOR_IGNITION, OUTPUT);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    tvc.begin();
    tvc.lock();

    Serial.println("Initialized");
    Serial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");

    Serial.println(HELP_STR);
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

    tvc.update(dir);
    dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, yaw, pitch, biases);
    yaw = dir.yaw;
    pitch = dir.pitch;

    if (newData == true) {
        switch (receivedChar) {
            case 'U':
                Serial.println("Unlocking TVC");
                tvc.unlock();
                break;
            case 'L':
                Serial.println("Locking TVC");
                tvc.lock();
                break;
            case 'C':
                Serial.println("Calibrating Gyro");
                calibrateGyro();
                break;
            case 'O':
                Serial.print("Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                Serial.print("Altitude: ");
                Serial.println(getAltitude());
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
                fire_pyro_2(PYRO_LANDING_MOTOR_IGNITION);
                break;
            case 'T':
                Serial.println("Pyro 2: Landing Legs Deploy");
                fire_pyro_2(PYRO_LANDING_LEGS_DEPLOY);
                break;
            case 'S':
                Serial.println("Reading SD Card Info");
                sdCardInfo();
                break;
            case 'Q':
                Serial.println("Resetting Angles");
                yaw = 0;
                pitch = 0;
                break;
            case 'H':
                Serial.println();
                break;


            default:
                Serial.println("Invalid command");
                break;
        }
        newData = false;
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

    DELTA_TIME = (micros() - lastMicros) / 1000000.0;
    lastMicros = micros();
}