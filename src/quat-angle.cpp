#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>
#include <Quaternion.h>

SensorReadings readings;
Vec2D dir;
Biases biases;
Quaternion attitude;

double yaw, pitch, roll;

bool newCommand = false;
char receivedChar;

double norm;

void setup() {
    Serial.begin(115200);
    delay(200);
    initIMU();

    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateSensors();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);

    attitude = Quaternion();  // pointing forward and flat
    pinMode(3, INPUT_PULLUP);

}

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

long long lastLoopTime = micros();
void loop() {
    // --- Read Sensors --- //
    readSensors(readings, biases);

    recvOneChar();

    if (newCommand) {
        switch (receivedChar) {
            case 'Q':
                attitude = Quaternion();
                dir = Vec2D(0, 0);
                yaw = 0;
                pitch = 0;
                roll = 0;
                // biases = calibrateSensors();
                break;
        }


        newCommand = false;
    }

    // Init Gyro Rate Quaternion
    // X = pitch, Y = roll, Z = yaw
    double wx = readings.gx * (PI / 180);
    double wy = readings.gy * (PI / 180);
    double wz = readings.gz * (PI / 180);


    // Update attitude quaternion
    norm = sqrt(wx * wx + wy * wy + wz * wz);
    norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0

    wx /= norm;
    wy /= norm;
    wz /= norm;

    Quaternion QM = Quaternion::from_axis_angle(DELTA_TIME * norm, wx, wy, wz);
    attitude = attitude * QM;

    // --- Convert to Euler Angles --- //    
    // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
    double qw = attitude.a;
    double qz = attitude.b;
    double qx = attitude.c;
    double qy = attitude.d;

    // from https://www.euclideanspace.com/maths/standards/index.htm
    if (qx * qy + qz * qw >= 0.5) {  // North pole
        yaw = 2 * atan2(qx, qw);
        pitch = PI / 2;
        roll = 0;
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (qx * qy + qz * qw <= -0.5) {  // South pole
        yaw = -2 * atan2(qx, qw);
        pitch = -PI / 2;
        roll = 0;
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz);
        pitch = asin(2 * qx * qy + 2 * qz * qw);
        roll = atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz);
        digitalWrite(LED_BUILTIN, LOW);
    }

    attitude = attitude.normalize();


    pitch *= 180 / PI;
    yaw *= 180 / PI;
    roll *= 180 / PI;

    Serial.print(qw);
    Serial.print(" ");
    Serial.print(qx);
    Serial.print(" ");
    Serial.print(qy);
    Serial.print(" ");
    Serial.print(qz);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(yaw);

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

    delay(2);
    DELTA_TIME = (micros() - lastLoopTime) / 1000000.;
    lastLoopTime = micros();
}