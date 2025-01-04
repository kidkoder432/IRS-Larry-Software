// Test all components and features

#include <Arduino.h>
#include <ESP32Servo.h>

#include <orientation.h>

#include <pins.h>
#include <tvc.h>
#include <leds.h>
#include <pyro.h>


#include <config.h>
#include <datalog.h>


float yaw, pitch, roll;
TVC tvc;
float x_out, y_out;
long long lastLoopTime;

char receivedChar;
bool newCommand = false;

double deltaTime = 0.01;

Config config;

bool logData = true;
bool led = true;

SdExFat SD;

ExFile dataFile;
ExFile logFile;

int currentState = 42;   // Test state = 42

const char HELP_STR[] =
R"(Input commands to test the following:
Y: Toggle LEDs
D: Toggle Data Logging
G: Activate Pyro 1 (Motor Ignition)
T: Activate Pyro 2 (Landing Legs Deploy)
S: Read SD Card

TVC Commands:
I: Nudge TVC Up
J: Nudge TVC left
K: Nudge TVC Down
L: Nudge TVC Right
X: Full TVC Test (will block until done)
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
    pinMode(PYRO_1_LANDING_MOTOR_PIN, OUTPUT);
    pinMode(PYRO_2_LANDING_LEGS_PIN, OUTPUT);

    // Init LEDs
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    // Init SD Card | Read Config
    SD.begin(10);
    // logFile.open("/log.txt", FILE_WRITE);

    // if (!logFile) {
    //     Serial.println("Failed to open log file!");
    //     while (1) {
    //         flash(COLOR_RED);

    //     }
    // }

    // logStatus("Initialized", logFile);
    // config = readConfig();
    // logStatus("Config read successfully", logFile);


    // // Open data file
    // dataFile.open("/data.csv", FILE_WRITE);

    // if (!dataFile) {
    //     Serial.println("Failed to open data file!");
    //     while (1) {
    //         flash(COLOR_RED);

    //     }
    // }
    // dataFile.println("Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Xout,Yout,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");

    // Init TVC
    tvc.begin(deltaTime);
    tvc.lock();

    delay(1000);

    Serial.println("Initialized ESP32");
    Serial.println(R"(Welcome to Larry v1 Interactive Test Suite.
This test suite will test all components and features of the flight computer.)");

    delay(200);
    lastLoopTime = micros();

}

void loop() {


    recvOneChar();

    if (newCommand == true) {
        switch (receivedChar) {

            case 'G':
                Serial.println("Pyro 1: Motor Ignition");
                logStatus("Pyro 1: Motor Ignition", logFile);
                fire_pyro_test(PYRO_1_LANDING_MOTOR_PIN);
                break;
            case 'T':
                Serial.println("Pyro 2: Landing Legs Deploy");
                logStatus("Pyro 2: Landing Legs Deploy", logFile);
                fire_pyro_test(PYRO_2_LANDING_LEGS_PIN);
                break;
            case 'S':
                Serial.println("Reading SD Card Info");
                logStatus("Reading SD Card Info", logFile);
                sdCardInfo(SD);
                break;
            case 'H':
                Serial.println(HELP_STR);
                break;

            case 'D':
                logData = !logData;
                if (logData) {
                    // dataFile.open("/data.csv", FILE_WRITE);
                    // logFile.open("/log.txt", FILE_WRITE);
                    Serial.println("Logging Data ON");
                    logStatus("Logging Data ON", logFile);
                }
                else {
                    Serial.println("Logging Data OFF");
                    logStatus("Logging Data OFF", logFile);
                    // dataFile.close();
                    // logFile.close();
                }
                break;

            case 'I':
                Serial.println("Nudging TVC Up");
                logStatus("Nudging TVC Up", logFile);
                tvc.nudge(2);
                break;

            case 'J':
                Serial.println("Nudging TVC Left");
                logStatus("Nudging TVC Left", logFile);
                tvc.nudge(1);
                break;

            case 'K':
                Serial.println("Nudging TVC Down");
                logStatus("Nudging TVC Down", logFile);
                tvc.nudge(3);
                break;

            case 'L':
                Serial.println("Nudging TVC Right");
                logStatus("Nudging TVC Right", logFile);
                tvc.nudge(0);
                break;
            
            case 'X':
                Serial.println("Running TVC Test");
                logStatus("Running TVC Test", logFile);
                tvc.testRoutine();
                Serial.println("TVC Test Complete");
                logStatus("TVC Test Complete", logFile);
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
        p.timestamp = lastLoopTime;
        p.DELTA_T = deltaTime;
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


    deltaTime = (micros() - lastLoopTime) / 1000000.0;
    lastLoopTime = micros();
}
