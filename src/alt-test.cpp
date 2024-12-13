#include <rocket.h>

Rocket rocket;

float alpha = 0.95;
float alpha_alt = 0.95;

float newAlt, vel;
float prevBaroAlt_filtered = 0;
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

    float baroAlt_raw = rocket.getAlt();
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
    float baroAlt_filtered = lerp(prevBaroAlt_filtered, baroAlt_raw, 0.7);

    float baroVel = (baroAlt_filtered - prevBaroAlt_filtered) / rocket.deltaTime;
    vel = lerp(baroVel, velAccel, alpha);

    prevBaroAlt_filtered = baroAlt_filtered;


    float altAccel = newAlt + vel * rocket.deltaTime;
    altAccel_raw += vel * rocket.deltaTime;
    newAlt = lerp(baroAlt_filtered, altAccel, alpha_alt);

    rocket.printMessage(ay, false);
    rocket.printMessage(baroAlt_raw, false);
    rocket.printMessage(baroAlt_filtered, false);
    rocket.printMessage(newAlt);


}