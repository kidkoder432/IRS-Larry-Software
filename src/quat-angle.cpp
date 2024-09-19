#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>
#include <Quaternion.h>

SensorReadings readings, prevReadings;
Vec2D dir;
Biases biases;
Quaternion attitude;

double yaw, pitch, roll;
double yaw_gyr, pitch_gyr;

bool newCommand = false;
char receivedChar;

float norm;

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

long long lastMicros = micros();
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
                yaw_gyr = 0;
                pitch_gyr = 0;
                // biases = calibrateSensors();
                break;
        }


        newCommand = false;
    }

    // Init Gyro Rate Quaternion
    // X = roll, Y = pitch, Z = yaw
    double wx = readings.gx * (PI / 180);
    double wy = readings.gy * (PI / 180);
    double wz = readings.gz * (PI / 180);

    norm = sqrt(wx * wx + wy * wy + wz * wz);

    norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0

    wx /= norm; 
    wy /= norm;
    wz /= norm;

    Quaternion QM = Quaternion::from_axis_angle(DELTA_TIME * norm, wx, wy, wz);

    // Quaternion QM = Quaternion::from_euler_rotation(wz, wy, wx);

    // Quaternion multiplication
    attitude *= QM;

    // Convert to Euler angles

    float sinr_cosp = 2 * (attitude.a * attitude.b + attitude.c * attitude.d);
    float cosr_cosp = 1 - 2 * (attitude.b * attitude.b + attitude.c * attitude.c);
    roll = atan2(sinr_cosp, cosr_cosp);


    float sinp = 2.0f * (attitude.a * attitude.c - attitude.b * attitude.d);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2, sinp); // return 90 if out of range
    else
        pitch = asin(sinp);

    float siny_cosp = 2 * (attitude.a * attitude.d + attitude.b * attitude.c);
    float cosy_cosp = 1 - 2 * (attitude.c * attitude.c + attitude.d * attitude.d);
    yaw = atan2(siny_cosp, cosy_cosp);

    attitude = attitude.normalize();

    dir = get_angles(readings, prevReadings, dir, DELTA_TIME);

    yaw_gyr = dir.x;
    pitch_gyr = dir.y;

    pitch *= 180 / PI;
    yaw *= 180 / PI;
    roll *= 180 / PI;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

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
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    prevReadings = readings;
    lastMicros = micros();
}