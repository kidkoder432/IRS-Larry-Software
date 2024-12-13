#include <rocket-noble.h>

Rocket rocket;

float newAlt, vel;
float prevBaroAlt = 0;
float altAccel_raw = 0;

float lerp(float a, float b, float t) { return a + (b - a) * t; }

void setup() {
    rocket.initSerial();
    rocket.initSD();
    rocket.initLogs();
    rocket.initConfig();
    rocket.setupSensors();
    rocket.calibrateAndLog();
    rocket.initAngles();
    rocket.initLeds();
    rocket.finishSetup();

}

void loop() {

    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();
    rocket.updateAngleLeds();
    rocket.updateTime();

    float alt = rocket.getAlt();
    Quaternion attitude = rocket.getAttitude();
    SensorReadings readings = rocket.getReadings();

    float ax = readings.ax;
    float ay = readings.ay;
    float az = readings.az;

    Quaternion Qaccel = Quaternion(0, ax, ay, az);

    Quaternion Qaccel_world = (-attitude * Qaccel) * -attitude.conj();

    float ayMps = (Qaccel_world.c - 1) * 9.81f;

    // vel = vel - ayMps * rocket.deltaTime;

    float velAccel = vel - ayMps * rocket.deltaTime;
    float baroAlt = lerp(prevBaroAlt, alt, 0.8);

    float baroVel = (baroAlt - prevBaroAlt) / rocket.deltaTime;
    vel = lerp(baroVel, velAccel, rocket.getConfigValue("ALT_FILTER_ALPHA_ACCEL"));

    prevBaroAlt = alt;


    float altAccel = newAlt + vel * rocket.deltaTime;
    altAccel_raw += vel * rocket.deltaTime;
    newAlt = lerp(baroAlt, altAccel, rocket.getConfigValue("ALT_FILTER_ALPHA_ACCEL"));

    rocket.printMessage(baroAlt, false);
    rocket.printMessage(altAccel, false);
    rocket.printMessage(alt, false);
    rocket.printMessage(newAlt);


}