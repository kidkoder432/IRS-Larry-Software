#include <Wire.h>
#include <orientation.h>
#include <sensors.h>
#include <Arduino.h>
#include <config.h>


#define LPS22HB_ADDRESS  0x5C

#define LPS22HB_WHO_AM_I_REG        0x0f
#define LPS22HB_CTRL1_REG           0x10
#define LPS22HB_CTRL2_REG           0x11
#define LPS22HB_STATUS_REG          0x27
#define LPS22HB_PRESS_OUT_XL_REG    0x28
#define LPS22HB_PRESS_OUT_L_REG     0x29
#define LPS22HB_PRESS_OUT_H_REG     0x2a
#define LPS22HB_TEMP_OUT_L_REG      0x2b
#define LPS22HB_TEMP_OUT_H_REG      0x2c

enum {
    RATE_ONE_SHOT = 0,
    RATE_1_HZ = 1,
    RATE_10_HZ = 2,
    RATE_25_HZ = 3,
    RATE_50_HZ = 4,
    RATE_75_HZ = 5,
};

class LPSBaro {
public:
    LPSBaro(TwoWire& i2c) : i2c(i2c) {}

    bool i2cWrite(uint8_t reg, uint8_t val) {
        this->i2c.beginTransmission(LPS22HB_ADDRESS);
        this->i2c.write(reg);
        this->i2c.write(val);
        if (i2c.endTransmission() != 0) {
            return false;
        }
        return true;
    }

    int i2cRead(uint8_t reg) {
        this->i2c.beginTransmission(LPS22HB_ADDRESS);
        this->i2c.write(reg);
        if (this->i2c.endTransmission(false) != 0) {
            return -1;
        }

        if (this->i2c.requestFrom(LPS22HB_ADDRESS, 1) != 1) {
            return -1;
        }

        return this->i2c.read();
    }

    bool begin() {
        this->i2c.begin();

        if (i2cRead(LPS22HB_WHO_AM_I_REG) != 0xb1) {
            end();
            return false;
        }

        initialized = true;
        return true;
    }

    void end() {
        this->i2c.end();
        this->initialized = false;
    }

    bool config(int rate) {

        int err = 0;
        if (rate == RATE_ONE_SHOT) {
            return false;
        }

        this->rate = rate;
        int BDU = 1;
        uint8_t val = ((rate & 0x07) << 4) | (BDU << 1);
        err += i2cWrite(LPS22HB_CTRL1_REG, val);

        return err;
    }

    bool dataReady() {
        return (i2cRead(LPS22HB_STATUS_REG) & 0x01);
    }

    float readPressure() {

        if (!this->initialized) {
            return 0;
        }

        if (dataReady()) {

            uint8_t reg_xl = i2cRead(LPS22HB_PRESS_OUT_XL_REG);
            uint8_t reg_l = i2cRead(LPS22HB_PRESS_OUT_L_REG);
            uint8_t reg_h = i2cRead(LPS22HB_PRESS_OUT_H_REG);

            // 1. Shift up to copy the sign bit
            int32_t currentReading = (reg_h << 24) | (reg_l << 16) | (reg_xl << 8);

            // 2. Shift exactly 8 bits back down to fix the 24-bit alignment
            currentReading = currentReading >> 8;

            // 3. Store it as a FLOAT and use decimal division to keep the fractional hPa
            currentPressure_hpa = currentReading / 4096.0f;

        }

        return currentPressure_hpa;
    }

private:
    TwoWire& i2c;
    bool initialized = false;
    int rate = RATE_75_HZ;
    float currentPressure_hpa = 0.0f;

};

class Altimeter {
public:
    Altimeter() : baro(Wire1) {
        currentAlt = 0;
        currentVel = 0;

    }

    bool begin() {
        if (!baro.begin()) {
            Serial.println("Failed to initalize Baro");
            return false;
        }
        baro.config(RATE_75_HZ);
        Serial.println("Calibrating baro...");
        calibrate();
        Serial.println("Calibration done!");
        return true;
    }

    void update(const SensorReadings& r, Quaternion a, float dt) {
        Vec3D accel = accel_body_to_world(r, a);
        float accelX = (accel.x - 1.0f) * 9.81f;
        accelVel = currentVel + accelX * dt;
        accelAlt = currentAlt + accelVel * dt + 0.5f * accelX * dt * dt;


        baroAlt = getBaroAltitude(baro.readPressure());

        deviation = baroAlt - accelAlt;
        currentAlt = accelAlt + deviation * alpha;
        currentVel = accelVel + deviation * beta;
    }

    float getAltitude() { return currentAlt; }
    float getVelocity() { return currentVel; }

    void reset() {
        currentAlt = 0;
        currentVel = 0;
    }

    Vec3D accel_body_to_world(const SensorReadings& r, Quaternion a) {
        Quaternion accel_q = Quaternion(0, r.ax, r.ay, r.az);
        Quaternion accel_global = a * accel_q * a.conj();

        return Vec3D(accel_global.b, accel_global.c, accel_global.d);
    }

    void printData() {
        char buf[256];
        snprintf(buf, sizeof(buf), ">alt:%.3f,vel:%.3f,accelAlt:%.3f,accelVel:%.3f,baroAlt:%.3f,deviation:%.3f;", currentAlt, currentVel, accelAlt, accelVel, baroAlt, deviation);
        Serial.println(buf);
    }

    void calibrate() {
        unsigned long long start = micros();

        float totalPressure = 0.0f;
        int count = 0;

        while (micros() - start < 3000000) {
            if (baro.dataReady()) {
                totalPressure += baro.readPressure();
                count++;
            }
        }

        pressure_ref = totalPressure / count;

    }

private:

    float getBaroAltitude(float currentPressure) {
        return 44330.0f * (1.0f - pow(currentPressure / pressure_ref, 0.19026f));
    }

    LPSBaro baro;

    float currentAlt = 0, currentVel = 0;

    float pressure_ref = 1013.25;
    float alpha = 0.05; // Trust the barometer this much
    float beta = 0.1; // Nudge the accelerometer from the barometer by this much

    float accelVel, accelAlt, baroAlt, deviation;

};