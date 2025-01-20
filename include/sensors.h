#ifndef SENSORS_H
#define SENSOR_H

#include <config.h>

#if USE_BLE_SENSE
#include <alt.h>
#endif

#if USE_RP2040
#include <SparkFunLSM6DSO.h>
#else
#include <SparkFun_BMI270_Arduino_Library.h>
#endif

#if USE_RP2040
LSM6DSO rp_imu;
#else
BMI270 imu;
#endif

// ========= Sensor Variables ========= //
struct SensorReadings {
    float ax, ay, az;
    float gx, gy, gz;

    SensorReadings() { ax = ay = az = 0; gx = gy = gz = 0; }

};

struct GyroBiases {
    float bx, by, bz;
    GyroBiases() { bx = by = bz = 0; };
    GyroBiases(float x, float y, float z) : bx(x), by(y), bz(z) {}

};

#if USE_RP2040
void initSensors() {
    Wire.begin();
    rp_imu.begin();
    rp_imu.setAccelRange(8);
    rp_imu.setAccelDataRate(416);
    rp_imu.setGyroRange(1000);
    rp_imu.setGyroDataRate(416);
    rp_imu.setBlockDataUpdate(true);
}

void readSensors(SensorReadings& r, GyroBiases biases) {
    r.ax = rp_imu.readFloatAccelX();
    r.ay = rp_imu.readFloatAccelY();
    r.az = rp_imu.readFloatAccelZ();

    r.gx = rp_imu.readFloatGyroX() - biases.bx;
    r.gy = rp_imu.readFloatGyroY() - biases.by;
    r.gz = rp_imu.readFloatGyroZ() - biases.bz;

}

GyroBiases calibrateSensors(Config& config) {

    Serial.println("Starting Sensor Calibration...");

    long long now = micros();
    float x_angle_c, y_angle_c, z_angle_c;
    x_angle_c = y_angle_c = z_angle_c = 0;
    float dt = 0.005;
    SensorReadings r;
    long long lastM = micros();
    while (micros() - now < 3000000LL) {

        r.gx = rp_imu.readFloatGyroX();
        r.gy = rp_imu.readFloatGyroY();
        r.gz = rp_imu.readFloatGyroZ();
        x_angle_c += dt * r.gx;
        y_angle_c += dt * r.gy;
        z_angle_c += dt * r.gz;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    float bx = ((x_angle_c) / (float)(micros() - now)) * 1000000;
    float by = ((y_angle_c) / (float)(micros() - now)) * 1000000;
    float bz = ((z_angle_c) / (float)(micros() - now)) * 1000000;

    return GyroBiases(bx, by, bz);
}

#else
bool initSensors() {
    Wire1.begin();
    if (!imu.beginI2C(0x68, Wire1)) return false;

    int err;

    // Configure accelerometer
    bmi2_sens_config accConfig;
    accConfig.type = BMI2_ACCEL;
    accConfig.cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    accConfig.cfg.acc.range = BMI2_ACC_RANGE_16G;
    accConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(accConfig);

    // Configure gyroscope
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err += imu.setConfig(gyroConfig);

    if (err) {
        Serial.println("Error setting sensor configuration.");
        return false;
    }

#if USE_BLE_SENSE
    if (!BARO.begin()) return false;
    BARO.setOutputRate(RATE_75_HZ);
#endif

    return true;

}

void readSensors(SensorReadings& r, GyroBiases biases) {
    imu.getSensorData();

    r.ax = imu.data.accelX;
    r.ay = imu.data.accelY;
    r.az = imu.data.accelZ;

    r.gx = imu.data.gyroX - biases.bx;
    r.gy = imu.data.gyroY - biases.by;
    r.gz = imu.data.gyroZ - biases.bz;
}

GyroBiases calibrateSensors(Config& config) {

    Serial.println("Starting Sensor Calibration...");

    // Accelerometer + gyroscope CRT calibration
    if (config["DO_CRT"] > 0) {
        Serial.println("Performing component retrimming...");
        imu.performComponentRetrim();
    }

    // Accelerometer FOC calibration
    if (config["DO_ACCEL_FOC"] > 0) {
        Serial.println("Performing acclerometer offset calibration...");
        imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Y);
    }

    // Gyroscope FOC calibration
    if (config["DO_GYRO_FOC"] > 0) {
        Serial.println("Performing gyroscope offset calibration...");
        imu.performGyroOffsetCalibration();
    }

    Serial.println();
    Serial.println("Internal calibration complete!");
    Serial.println("Starting external gyroscope offset calibration...");

    long long now = micros();
    float x_angle_c, y_angle_c, z_angle_c;
    x_angle_c = y_angle_c = z_angle_c = 0;
    float dt = 0.005;
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

    float bx = ((x_angle_c) / (float)(micros() - now)) * 1000000;
    float by = ((y_angle_c) / (float)(micros() - now)) * 1000000;
    float bz = ((z_angle_c) / (float)(micros() - now)) * 1000000;

    return GyroBiases(bx, by, bz);
}
#endif

#endif