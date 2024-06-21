#include "math.h"

#include "Arduino.h"
#include "Serial.h"
#include "Servo.h"

#include "ArduPID.h"
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include "Kalman.h"
#include <leds.h>
#include <orientation.h>
#include <pyro.h>
#include <datalog.h>  // TODO: Implement
#include <alt.h>


// --------- Helper functions --------- //
double mag3(double a, double b, double c) {
    return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
}

// --------- TVC --------- //
Servo tvcx, tvcy;

// placeholder values; replace with actual limits
const double XMIN = 80;  // TVC X Min
const double XMAX = 100; // TVC X Max
const double YMIN = 80;  // TVC Y Min
const double YMAX = 100; // TVC Y Max
const double XDEF = 90;  // TVC X Default (zero position)
const double YDEF = 90;  // TVC Y Default (zero position)

// --------- TVC Control --------- //
ArduPID pid_x;
ArduPID pid_y;
const double P = 8.58679935818825;
const double I = 12.4428493210038;
const double D = 0.482664861486399;

double yaw;
double pitch;

double x_out;
double y_out;

// --------- Sensor Variables --------- //
struct SensorReadings readings;
struct Orientation dir, m_dir;

double altitude, maxAltitude = -10000000;

// --------- Filter Init --------- //
Kalman kx, ky;
Biases biases;

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

// --------- States --------- //
int currentState = 0;

void setup() {
    Serial.begin(9600);
    IMU.begin();
    BARO.begin();

    tvcx.attach(5);
    tvcy.attach(6);

    pid_x.begin(&yaw, &x_out, 0, P, I, D);
    pid_x.setOutputLimits(XMIN, XMAX);
    pid_x.setWindUpLimits(XMIN, XMAX);
    tvcx.write(XDEF);

    pid_y.begin(&pitch, &y_out, 0, P, I, D);
    pid_y.setOutputLimits(YMIN, YMAX);
    pid_y.setWindUpLimits(YMIN, YMAX);
    tvcy.write(YDEF);

    delay(500);


    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    Serial.println(atan2(ax, az) * 180 / PI);
    yaw = atan2(ay, az) * 180 / PI - 90;
    pitch = atan2(ax, az) * 180 / PI - 90;

    kx.setAngle(yaw);
    ky.setAngle(pitch);

    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateGyro();

    pinMode(PYRO_LANDING_LEGS_DEPLOY, OUTPUT);
    pinMode(PYRO_LANDING_MOTOR_IGNITION, OUTPUT);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    showColor(COLOR_OFF);

    logTimer.begin(1000L / PAD_IDLE_LOG_FREQ);
}

// --- PID Control loops --- //
// TODO: Tune
void PID(Orientation dir) {
    // X
    yaw = dir.yaw;
    pid_x.compute();
    tvcx.write(x_out);

    // Y
    pitch = dir.pitch;
    pid_y.compute();
    tvcy.write(y_out);

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
    dir = get_angles_kalman(DELTA_TIME, readings, kx, ky, biases);

    Serial.print(yaw);
    Serial.print(" -x  y- ");
    Serial.println(pitch);

    // --- LED Control --- //
    LED(currentState);

    // --- States --- //
    switch (currentState) {
        case 0:  // Pad-Idle
            logTimer.update(1000L / PAD_IDLE_LOG_FREQ);
        case 1:  // Powered Ascent
            logTimer.update(1000L / FLIGHT_LOG_FREQ);
            PID(dir);

        case 2:  // Coast Up
            break; // Not used (for now)
        case 3:  // Coast (Down)
            tvcx.write(XDEF);
            tvcy.write(YDEF);
            break;
        case 4:  // Powered Descent
            PID(dir);
            break;
        case 5:  // Touchdown
            logTimer.update(1000L / PAD_IDLE_LOG_FREQ);
            break;
    }

    // --- Pyro Channels --- //

    fire_pyro(pt1, PYRO_LANDING_MOTOR_IGNITION);
    fire_pyro(pt2, PYRO_LANDING_LEGS_DEPLOY);


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
