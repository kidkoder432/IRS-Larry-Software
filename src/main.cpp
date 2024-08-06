#include "math.h"

#include "Arduino.h"
#include "Serial.h"
#include "Servo.h"

#include "ArduPID.h"
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include <leds.h>
#include <orientation.h>
#include <pyro.h>
#include <datalog.h>  // TODO: Implement
#include <alt.h>
#include "tvc.h"


// --------- Helper functions --------- //
double mag3(double a, double b, double c) {
    return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
}

// --------- TVC --------- //
TVC tvc;
double x_out, y_out;

// --------- Sensor Variables --------- //
struct SensorReadings readings;
struct Orientation dir;


double altitude, maxAltitude = -10000000;

// --------- Filter Init --------- //
Biases biases;
float ALPHA = 0.05;

// --------- Landing --------- //
const double ALT_LAND_ENGINE_START = 15; // meters AGL to start land burn
const double LANDING_LEGS_DEPLOY_DELAY = 1300; // ms after engine ignition to deploy legs

// --------- Pyro Channels --------- //
const long ENGINE_START_PYRO_ON = 1000;
const long LANDING_DEPLOY_PYRO_ON = 1500;
bool pyro1Arm = true;
bool pyro2Arm = true;
FireTimer pt1, pt2;

// --------- Data Logging --------- //
const int PAD_IDLE_LOG_FREQ = 10;   // Pad-Idle logs @ 10 Hz
const int FLIGHT_LOG_FREQ = 40;     // Inflight logs @ 40 Hz
FireTimer logTimer;
File dataFile;
char filename[64] = "data.csv";
float vertVel;
// --------- States --------- //
int currentState = 0;

void setup() {
    Serial.begin(9600);
    IMU.begin();
    BARO.begin();

    tvc.begin();
    tvc.lock();

    delay(500);


    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    Serial.println(atan2(ax, az) * 180 / PI);
    dir.yaw = atan2(ay, az) * 180 / PI - 90;
    dir.pitch = atan2(ax, az) * 180 / PI - 90;

    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateGyro();

    pinMode(PYRO_LANDING_LEGS_DEPLOY, OUTPUT);
    pinMode(PYRO_LANDING_MOTOR_IGNITION, OUTPUT);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    logTimer.begin(1000L / PAD_IDLE_LOG_FREQ);
    tvc.unlock();
}

// Main Control Loop
long long lastMicros = micros();
void loop() {

    lastMicros = micros();

    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);

    altitude = getAltitude();
    maxAltitude = max(altitude, maxAltitude);

    // --- Angle Calc --- //
    dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, dir.yaw, dir.pitch, biases);
    
    // TVC Update
    Orientation tvc_out = tvc.update(dir, DELTA_TIME);
    Orientation tvc_out = tvc.update(dir, DELTA_TIME);
    x_out = tvc_out.yaw;
    y_out = tvc_out.pitch;

    Serial.print(dir.yaw);
    Serial.print(" -x  y- ");
    Serial.println(dir.pitch);

    // --- LED Control --- //
    LED(currentState);

    // --- States --- //
    switch (currentState) {
        case 0:  // Pad-Idle
            tvc.lock();
            logTimer.update(1000L / PAD_IDLE_LOG_FREQ);
            break;
        case 1:  // Powered Ascent
            tvc.unlock();
            logTimer.update(1000L / FLIGHT_LOG_FREQ);
            break;
        case 2:  // Coast Up
            break; // Not used (for now)
        case 3:  // Coast (Down)
            tvc.lock();
            break;
        case 4:  // Powered Descent
            tvc.unlock();
            break;
        case 5:  // Touchdown
            logTimer.update(1000L / PAD_IDLE_LOG_FREQ);
            break;
    }

    // --- Pyro Channels --- //

    fire_pyro(pt1, PYRO_LANDING_MOTOR_IGNITION);
    fire_pyro(pt2, PYRO_LANDING_LEGS_DEPLOY);


    // --- Pyro Timers --- //

    long long msFire = 0;
    if (abs(altitude - ALT_LAND_ENGINE_START) <= 0.07 && pyro1Arm) {
        msFire = millis();
        pt2.begin(ENGINE_START_PYRO_ON);
        pyro1Arm = false;

    }

    if ((millis() - msFire) >= LANDING_LEGS_DEPLOY_DELAY && pyro2Arm) {
        pt2.begin(LANDING_DEPLOY_PYRO_ON);
        pyro2Arm = false;

    }

    // --- Data Logging --- //
    vertVel += readings.ay * 9.81 * DELTA_TIME;
    if (logTimer.fire()) {
        dataFile = SD.open(filename, FILE_WRITE);
        DataPoint p;
        p.timestamp = micros();
        p.r = readings;
        p.o = dir;
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = getAltitude();
        p.currentState = currentState;
        p.vert_vel = vertVel;
        p.kp = tvc.pid_x.Kp;
        p.ki = tvc.pid_x.Ki;
        p.kd = tvc.pid_x.Kd;

        logDataPoint(p, dataFile);

        // if (millis() % 100 == 0) {
        //     dataFile.flush();
        // }
        dataFile.close();
    }


    // --- State Transitions --- //

    // TOUCHDOWN: State 4 -> 5
    if (currentState == 4 && mag3(readings.ax, readings.ay, readings.az) <= 1.5) {
        currentState = 5;
    }

    // STAGE 2 IGNITION: State 3 -> 4
    if (currentState == 3 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
        currentState = 4;
    }

    // APOGEE: State 2 -> 3
    // if (currentState == 2 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
    //     currentState = 3;

    // }

    // STAGE 1 BURNOUT: State 1 -> 3
    if (currentState == 1 && mag3(readings.ax, readings.ay, readings.az) <= 1.5) {
        currentState = 3;
    }

    // LAUNCH: State 0 -> 1
    if (currentState == 0 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
        currentState = 1;
    }

    delay(10);
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
}
