// Test all components and features

#include <Arduino.h>
#include <ESP32Servo.h>

#include <orientation.h>

#include <tvc.h>
#include <leds.h>
#include <pyro.h>

#include <config.h>
#include <datalog.h>

double yaw, pitch, roll;
TVC tvc;
double x_out, y_out;
long long lastMicros;

char receivedChar;
bool newCommand = false;

Config config;

bool logData = true;
bool led = true;
File dataFile;
File logFile;

int currentState = 42;   // Test state = 42

const char HELP_STR[] =
R"(Input commands to test the following:
K: Toggle LEDs
D: Toggle Data Logging
G: Activate Pyro 1 (Motor Ignition)
T: Activate Pyro 2 (Landing Legs Deploy)
S: Read SD Card
H: Help)";

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        newCommand = true;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void setup() {

    // Init serial and sensors
    Serial.begin(115200);
    delay(2000);

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
    logFile = SD.open("log.txt", FILE_WRITE);

    if (!logFile) {
        Serial.println("Failed to open log file!");
        while (1) {
            flash(COLOR_RED);

        }
    }

    logStatus("Initialized", logFile);
    config = readConfig();
    logStatus("Config read successfully", logFile);


    // Open data file
    dataFile = SD.open("data.csv", FILE_WRITE);

    if (!dataFile) {
        Serial.println("Failed to open data file!");
        while (1) {
            flash(COLOR_RED);

        }
    }
    dataFile.println("Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Xout,Yout,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");

    // Init TVC
    tvc.begin();
    tvc.lock();

    delay(1000);

    Serial.println("Initialized ESP32");
    Serial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");

    delay(200);
    lastMicros = micros();

}

void loop() {


    recvOneChar();

    if (newCommand == true) {
        switch (receivedChar) {
            
            case 'K':
                Serial.println("Toggling LEDs");
                logStatus("Toggling LEDs", logFile);
                led = !led;
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
                checkSDCard();
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

    if (logData) {

        DataPoint p;
        p.timestamp = lastMicros;
        p.DELTA_T = DELTA_TIME;
        p.r = SensorReadings();
        p.o = Vec3D();
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = 0;
        p.currentState = currentState;
        p.vert_vel = 0;
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
