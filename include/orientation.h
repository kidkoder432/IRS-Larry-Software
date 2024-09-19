// ORIENTATION CALCULATION

#include <math.h>
// #include <Arduino_BMI270_BMM150.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Kalman.h>

// ========= Angles & Orientation ========= //

double DELTA_TIME = 0.01; // Time step
BMI270 imu;

// ========= Sensor Variables ========= //
struct SensorReadings {
    float ax, ay, az;
    float gx, gy, gz;

    SensorReadings() { ax = ay = az = 0; gx = gy = gz = 0; }

};

struct Vec2D {
    double x, y;

    Vec2D() { x = 0; y = 0; }
    Vec2D(double x, double y) : x(x), y(y) {}

};

int sign(double x) {
    if (x == 0) return 0;
    return x > 0 ? 1 : -1;
}

struct Biases {
    double bx, by, bz;
    Biases() { bx = by = bz = 0; };
    Biases(float x, float y, float z) : bx(x), by(y), bz(z) {}

};

void initIMU() {
    Wire1.begin();
    imu.beginI2C(0x68, Wire1);

    int err;

    // Remap axes
    bmi2_remap remap;
    remap.x = BMI2_AXIS_NEG_Y;
    remap.y = BMI2_AXIS_NEG_X;
    remap.z = BMI2_AXIS_POS_Z;
    imu.remapAxes(remap);

    // Configure accelerometer
    bmi2_sens_config accConfig;
    accConfig.type = BMI2_ACCEL;
    accConfig.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    accConfig.cfg.acc.range = BMI2_ACC_RANGE_16G;
    accConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(accConfig);
    Serial.println(err);

    // Configure gyroscope
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err += imu.setConfig(gyroConfig);
    Serial.println(err);

}

void readSensors(SensorReadings& r, Biases biases) {
    imu.getSensorData();

    r.ax = imu.data.accelX;
    r.ay = imu.data.accelY;
    r.az = imu.data.accelZ;

    r.gx = imu.data.gyroX - biases.bx;
    r.gy = imu.data.gyroY - biases.by;
    r.gz = imu.data.gyroZ - biases.bz;
}

Biases calibrateSensors() {

    Serial.println("Starting Sensor Calibration...");

    digitalWrite(LED_BUILTIN, HIGH);

    // Accelerometer + gyroscope CRT calibration
    Serial.println("Performing component retrimming...");
    imu.performComponentRetrim();

    // Accelerometer + gyroscope FOC calibration
    Serial.println("Performing acclerometer offset calibration...");
    // imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z);
    Serial.println("Performing gyroscope offset calibration...");
    // imu.performGyroOffsetCalibration();

    Serial.println();
    Serial.println("Internal calibration complete!");
    Serial.println("Starting static gyroscope offset calibration...");

    long long now = micros();
    double x_angle_c, y_angle_c, z_angle_c;
    x_angle_c = y_angle_c = z_angle_c = 0;
    double dt = 0.005;
    SensorReadings r;
    long long lastM = micros();
    while (micros() - now < 3000000LL) {
        imu.getSensorData();

        r.gx = imu.data.gyroX;
        r.gy = imu.data.gyroY;
        r.gz = imu.data.gyroZ;
        x_angle_c += dt * r.gx;
        y_angle_c += dt * r.gy;
        z_angle_c += dt * r.gz;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    Serial.println(y_angle_c);
    Serial.println(micros() - now);

    float bx = ((x_angle_c) / (double)(micros() - now)) * 1000000;
    float by = ((y_angle_c) / (double)(micros() - now)) * 1000000;
    float bz = ((z_angle_c) / (double)(micros() - now)) * 1000000;
    digitalWrite(LED_BUILTIN, LOW);

    return Biases(bx, by, bz);
}

// GYRO-BASED ANGLE CALCULATION
Vec2D get_angles(SensorReadings r, SensorReadings pr = SensorReadings(), Vec2D dir = Vec2D(0, 0), double dt = 0.02) {
    double x = dir.x - ((r.gz + pr.gz) * dt / 2);
    double y = dir.y + ((r.gy + pr.gy) * dt / 2);

    x = x * 0.85 + dir.x * 0.15;
    y = y * 0.85 + dir.y * 0.15;

    return Vec2D(x, y);
}

// COMPLEMENTARY FILTERING
Vec2D get_angles_complementary(double A, double dt, SensorReadings r, double yaw, double pitch, Biases b) {

    // Orientation w = local_to_global(r, b, yaw, pitch);
    Vec2D w = Vec2D(r.gz - b.bz, r.gx - b.bx);

    double accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    double gyro_angle_x = yaw - (w.x) * dt;
    double angle_x = accel_angle_x * (1.0 - A) + gyro_angle_x * A;

    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;
    double gyro_angle_y = pitch + (w.y) * dt;
    double angle_y = accel_angle_y * (1.0 - A) + gyro_angle_y * A;

    return Vec2D(angle_x, angle_y);
}


// KALMAN FILTERING
Vec2D get_angles_kalman(double dt, SensorReadings r, Kalman& kx, Kalman& ky, Biases b) {
    double accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;

    double kal_x = kx.getAngle(accel_angle_x, r.gz - b.bz, dt);
    double kal_y = ky.getAngle(accel_angle_y, r.gx - b.bx, dt);

    return Vec2D(kal_x, kal_y);

}

