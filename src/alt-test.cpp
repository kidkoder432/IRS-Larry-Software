#include <rocket.h>

Rocket rocket;

float lerp(float a, float b, float t) { return a + (b - a) * t; }

void setup() {
    rocket.initSerial();
    rocket.setupSensors();
    rocket.calibrateAndLog();
    rocket.initAngles();
    rocket.initLeds();
    rocket.finishSetup();

}


char receivedChar;
bool newCommand = false;
bool sensorOutLock = true;
bool dirOutLock = true;
bool altOutLock = true;
Altimeter& altimeter = rocket.altimeter;

long currentMs;

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#if USE_BLE 
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#endif
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void loop() {

    recvOneChar();
    if (newCommand) {
        switch (receivedChar) {
            case 'R':
                sensorOutLock = !sensorOutLock;
                break;
            case 'O':
                dirOutLock = !dirOutLock;
                break;
            case 'Q':
                rocket.resetAngles();
                altimeter.reset();
                break;
            case 'A':
                altOutLock = !altOutLock;
                break;
            case 'B':
                if (rocket.useCompl) {
                    rocket.disableCompl();
                }
                else {
                    rocket.enableCompl();
                }
        }
    }
    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();
    rocket.updateAngleLeds();
    rocket.updateTime();

    Altimeter altimeter = rocket.altimeter;

    Vec3D accel_world = altimeter.accel_body_to_world(rocket.getReadings(), rocket.getAttitude());

    SensorReadings r = rocket.getReadings();
    if (!sensorOutLock) {
        rocket.printMessage("Accel Body: ", false);
        rocket.printMessage(r.ax, false);
        rocket.printMessage(r.ay, false);
        rocket.printMessage(r.az);

        Quaternion attitude = rocket.getAttitude();
        rocket.printMessage("Q: ", false);
        rocket.printMessage(attitude.a, false);
        rocket.printMessage(attitude.b, false);
        rocket.printMessage(attitude.c, false);
        rocket.printMessage(attitude.d);


        rocket.printMessage("Accel World: ", false);
        rocket.printMessage(accel_world.x, false);
        rocket.printMessage(accel_world.y, false);
        rocket.printMessage(accel_world.z);
    }

    if (!altOutLock) {
        altimeter.printData();
    }


    if (!dirOutLock) {
        Vec3D dir = rocket.getDir();
        float roll = dir.x;
        float pitch = dir.y;
        float yaw = dir.z;

        rocket.printMessage("rpy: ", false);
        rocket.printMessage(roll, false);
        rocket.printMessage(pitch, false);
        rocket.printMessage(yaw);
    }

    // float baroAlt_raw = rocket.getAlt();
    // Quaternion attitude = rocket.getAttitude();
    // SensorReadings readings = rocket.getReadings();

    // float ax = readings.ax;
    // float ay = readings.ay;
    // float az = readings.az;

    // Quaternion Qaccel = Quaternion(0, ax, ay, az);

    // Quaternion Qaccel_world = (-attitude * Qaccel) * -attitude.conj();

    // float ayMps = (Qaccel_world.c - 1) * 9.81f;

    // // vel = vel - ayMps * rocket.deltaTime;

    // float velAccel = vel - ayMps * rocket.deltaTime;
    // float baroAlt_filtered = lerp(prevBaroAlt_filtered, baroAlt_raw, 0.7);

    // float baroVel = (baroAlt_filtered - prevBaroAlt_filtered) / rocket.deltaTime;
    // vel = lerp(baroVel, velAccel, alpha);

    // prevBaroAlt_filtered = baroAlt_filtered;


    // float altAccel = newAlt + vel * rocket.deltaTime;
    // altAccel_raw += vel * rocket.deltaTime;
    // newAlt = lerp(baroAlt_filtered, altAccel, alpha_alt);

    // rocket.printMessage(ay, false);
    // rocket.printMessage(baroAlt_raw, false);
    // rocket.printMessage(baroAlt_filtered, false);
    // rocket.printMessage(newAlt);


}