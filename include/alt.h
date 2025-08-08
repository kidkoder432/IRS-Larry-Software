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
    float baro_filter_alpha = 0.05;     // New: low-pass for baro altitude
    float vel_filter_alpha_baro = 0.05; // Blend for velocity fusion
    float alt_filter_alpha_baro = 0.05; // Blend for altitude fusion (baro)
    float alt_filter_alpha_vel = 0.2;  // Blend for altitude fusion (vel)

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
        vel_filter_alpha_baro = config["FILTER_BARO_VEL_ALPHA"] ? config["FILTER_BARO_VEL_ALPHA"] : vel_filter_alpha_baro;
        alt_filter_alpha_baro = config["FILTER_BARO_ALT_ALPHA"] ? config["FILTER_BARO_ALT_ALPHA"] : alt_filter_alpha_baro;
        alt_filter_alpha_vel = config["FILTER_VEL_ALT_ALPHA"] ? config["FILTER_VEL_ALT_ALPHA"] : alt_filter_alpha_vel;
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

    float getAccelAltitude(const Vec3D& accel_world, float prevAlt, float prevVel, float dt) {
        float newVel = getAccelVelocity(accel_world, prevVel, dt);
        return prevAlt + newVel * dt;
    }

    void update(const SensorReadings& r, const Quaternion& attitude, float dt) {
        // === 1. BARO BRANCH ===
        baroAlt_raw = getBaroAltitudeRaw();

        // EMA low-pass filter on baro altitude
        baroAlt_filt = baro_filter_alpha * baroAlt_raw + (1.0f - baro_filter_alpha) * baroAlt_filt;

        // Differentiate filtered baro altitude to get velocity
        baroVel_raw = (baroAlt_filt - prevBaroAlt_filt) / dt;

        // Low-pass baro velocity
        baroVel_filt = vel_filter_alpha_baro * baroVel_raw + (1.0f - vel_filter_alpha_baro) * baroVel_filt;

        prevBaroAlt_filt = baroAlt_filt;

        // === 2. ACCEL BRANCH ===
        Vec3D accel_world = accel_body_to_world(r, attitude);

        accelVel = getAccelVelocity(accel_world, prevAccelVel, dt);
        accelAlt = getAccelAltitude(accel_world, accelAlt, accelVel, dt);

        prevAccelVel = accelVel;

        // === 3. VELOCITY FUSION ===
        vel = (baroVel_filt * vel_filter_alpha_baro) + (accelVel * (1.0f - vel_filter_alpha_baro));

        // Integrate fused velocity for velocity-based altitude
        velAlt = alt + vel * dt;

        // === 4. ALTITUDE FUSION ===
        alt = (baroAlt_filt * alt_filter_alpha_baro)
            + (velAlt * alt_filter_alpha_vel)
            + (accelAlt * (1.0f - alt_filter_alpha_baro - alt_filter_alpha_vel));
    }

    float getAltitude() { return alt; }
    float getVelocity() { return vel; }

    void printData() {
        Serial.print(baroAlt_filt);
        Serial.print(" ");
        Serial.print(baroVel_filt);
        Serial.print(" ");
        Serial.print(accelAlt);
        Serial.print(" ");
        Serial.print(accelVel);
        Serial.print(" ");
        Serial.print(alt);
        Serial.print(" ");
        Serial.println(vel);
    }

    void reset() {
        pressureRef = calculateBasePressure();
        alt = vel = accelVel = accelAlt = velAlt = 0;
        baroAlt_filt = baroVel_filt = prevBaroAlt_filt = 0;
        prevAccelVel = 0;
    }
};

#endif
