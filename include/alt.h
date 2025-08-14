#ifndef ALT_H
#define ALT_H

#include <Arduino_LPS22HB.h>
#include <math.h>
#include <config.h>
#include <orientation.h>

class Altimeter {
private:
    // Config
    Config config;

    // Reference pressure
    float pressureRef;

    // Filter params (tunable)
    float baro_filter_alpha = 0.9;     // New: low-pass for baro altitude
    float alt_filter_alpha_baro = 0.2; // Blend for altitude fusion (baro)

    // States
    float alt = 0;    // Fused altitude
    float vel = 0;    // Fused velocity

    // Branch states
    float baroAlt_raw = 0;
    float baroAlt_filt = 0;
    float baroVel_raw = 0;
    float baroVel_filt = 0;

    float accelVel = 0;
    float accelAlt = 0;

    float velAlt = 0;

    // Prev states
    float prevBaroAlt_filt = 0;
    float prevAccelVel = 0;

    Quaternion attitude;

public:
    Altimeter() {}

    void configure(Config& cfg) {
        config = cfg;

        // Load from config if present
        alt_filter_alpha_baro = config["FILTER_BARO_ALT_ALPHA"] ? config["FILTER_BARO_ALT_ALPHA"] : alt_filter_alpha_baro;
        baro_filter_alpha = config["FILTER_BARO_EMA_ALPHA"] ? config["FILTER_BARO_EMA_ALPHA"] : baro_filter_alpha;

        pressureRef = calculateBasePressure();
    }

    float calculateBasePressure() {
        float avgPressure = 0;
        for (int i = 0; i < 50; i++) {
            avgPressure += BARO.readPressure(KILOPASCAL);
            delay(10);
        }
        avgPressure /= 50.0f;
        Serial.print("Base Pressure: ");
        Serial.println(avgPressure);
        return avgPressure;
    }

    float getBaroAltitudeRaw() {
        return 44330 * (1.0 - pow(BARO.readPressure(KILOPASCAL) / pressureRef, 0.1903));
    }

    Vec3D accel_body_to_world(const SensorReadings& r, const Quaternion& attitude) {
        Quaternion Qaccel = Quaternion(0, r.ax, r.ay, r.az);
        Quaternion Qaccel_global = (attitude * Qaccel) * attitude.conj();
        return Vec3D(Qaccel_global.b, Qaccel_global.c, Qaccel_global.d);
    }

    float getAccelVelocity(const Vec3D& accel_world, float prevVel, float dt) {
        // Gravity subtraction assumes perfect alignment
        return prevVel + (accel_world.x - 1.0f) * 9.81f * dt;
    }

    float getAccelAltitude(const Vec3D& accel_world, float prevAlt, float dt) {
        // float newVel = getAccelVelocity(accel_world, prevVel, dt);
        return prevAlt + (accel_world.x - 1.0f) * 9.81f * dt * dt;
    }

    void update(const SensorReadings& r, const Quaternion& attitude, float dt) {
        // === 1. BARO BRANCH ===
        baroAlt_raw = getBaroAltitudeRaw();

        // EMA low-pass filter on baro altitude
        baroAlt_filt = baro_filter_alpha * baroAlt_raw + (1.0f - baro_filter_alpha) * baroAlt_filt;

        // === 2. ACCEL BRANCH ===
        Vec3D accel_world = accel_body_to_world(r, attitude);

        // accelVel = getAccelVelocity(accel_world, prevAccelVel, dt);
        accelAlt = getAccelAltitude(accel_world, alt, dt);

        // === 4. ALTITUDE FUSION ===
        alt = (baroAlt_filt * alt_filter_alpha_baro)
            + (accelAlt * (1.0f - alt_filter_alpha_baro));
    }

    float getAltitude() { return alt; }
    float getVelocity() { return vel; }

    void printData() {
        Serial.print(baroAlt_filt);
        Serial.print(" ");
        Serial.print(accelAlt);
        Serial.print(" ");
        Serial.println(alt);
    }

    void reset() {
        pressureRef = calculateBasePressure();
        alt = vel = accelVel = accelAlt = velAlt = 0;
        baroAlt_filt = baroVel_filt = prevBaroAlt_filt = 0;
        prevAccelVel = 0;
    }
};

#endif
