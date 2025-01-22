#include <Arduino.h>
#include <pid.h>
#include <config.h>

#define ACTUAL_PID 1
#define PRINT_OUTPUT 1

float clamp(float x, float min, float max) { return (x < min) ? min : (x > max) ? max : x; }

class TVC {
public:

    PID pid_x;
    PID pid_y;

    void configure(Config& config) {
        P = config["Kp"];
        I = config["Ki"];
        D = config["Kd"];
        N = config["N"];

        XMIN = config["XMIN"];
        XMAX = config["XMAX"];
        YMIN = config["YMIN"];
        YMAX = config["YMAX"];

        XDEF = config["XDEF"];
        YDEF = config["YDEF"];

        pid_x.update_gains(P, I, D, N);
        pid_y.update_gains(P, I, D, N);

        FLIP_X = config["FLIP_X"] > 0;
        FLIP_Y = config["FLIP_Y"] > 0;
        ROLL_COMP = config["ROLL_COMP"] > 0;
    }

    void begin(float deltaTime) {
        dir = Vec3D(0, 0, 0);
        tvcx.attach(TVC_X_PIN);
        tvcy.attach(TVC_Y_PIN);

        pid_x.begin(P, I, D, XDEF, deltaTime, XMIN, XMAX);
        tvcx.write(XDEF);

        pid_y.begin(P, I, D, YDEF, deltaTime, YMIN, YMAX);
        tvcy.write(YDEF);
    }

    // X axis = yaw correction, Y axis = pitch correctin
    Vec2D update(Vec3D o, float dt) {
        if (!locked) {
            dir = o;
            // Serial.println(dir.z);
            // Serial.println(dir.y);
        #if ACTUAL_PID
            x_out = pid_x.update(0, dir.z, dt);
            y_out = pid_y.update(0, dir.y, dt);

        #else
            x_out = -dir.z;
            y_out = -dir.y;

        #endif




        #if PRINT_OUTPUT 
            Serial.print("X/Y Raw: ");
            Serial.print(x_out);
            Serial.print(" ");
            Serial.print(y_out);
            Serial.print("\t");
        #endif
            if (ROLL_COMP) {
                float tvcXRaw = (x_out)*PI / 180;
                float tvcYRaw = (y_out)*PI / 180;

                float cr = cos(dir.x * PI / 180);
                float sr = sin(dir.x * PI / 180);

                x_out = tvcXRaw * cr + tvcYRaw * sr;
                y_out = tvcXRaw * sr - tvcYRaw * cr;

                x_out = (x_out * 180 / PI);
                y_out = (y_out * 180 / PI);
            }

            if (FLIP_X) x_out = -x_out;
            if (FLIP_Y) y_out = -y_out;


        #if PRINT_OUTPUT

            x_out = clamp(x_out, XMIN, XMAX);
            y_out = clamp(y_out, YMIN, YMAX);
            Serial.print("X/Y Adjust: ");
            Serial.print(x_out);
            Serial.print(" ");
            Serial.println(y_out);
        #endif

            // Scale values less than defaults so that they have the full range of motion
            if (x_out < 0) {
                x_out *= (XDEF - XMIN) / (XMAX - XDEF);
            }

            if (y_out < 0) {
                y_out *= (YDEF - YMIN) / (YMAX - YDEF);
            }

            x_out += XDEF;
            y_out += YDEF;

            x_out = clamp(x_out, XMIN, XMAX);
            y_out = clamp(y_out, YMIN, YMAX);

            write(x_out, y_out);

            return Vec2D(x_out, y_out);

        }
        else {
            x_out = XDEF;
            y_out = YDEF;
            write(x_out, y_out);
            pid_x.reset();
            pid_y.reset();


            return Vec2D(XDEF, YDEF);
        }

    }

    // Writes angles to TVC
    void write(float x, float y) {
        if (x > XMAX) x = XMAX;
        if (x < XMIN) x = XMIN;
        if (y > YMAX) y = YMAX;
        if (y < YMIN) y = YMIN;
        tvcx.write(roundf(x));
        tvcy.write(roundf(y));

        x_out = x;
        y_out = y;
    }

    Vec2D getAngle() {
        return Vec2D(x_out, y_out);
    }

    Vec2D read() {
        return Vec2D(tvcx.read(), tvcy.read());
    }

    void lock() {
        locked = true;
    }

    void unlock() {
        tvcx.write(XDEF);
        tvcy.write(YDEF);
        locked = false;
    }

    void abort() {
        tvcx.write(XDEF);
        tvcy.write(YDEF);
        locked = true;
    }

    void nudge(int dir) {
        switch (dir) {
            case 0:
                x_out += 1;
                write(x_out, y_out);
                break;
            case 1:
                x_out -= 1;
                write(x_out, y_out);
                break;
            case 2:
                y_out += 1;
                write(x_out, y_out);
                break;
            case 3:
                y_out -= 1;
                write(x_out, y_out);
                break;
        }
    }

    void testRoutine() {
        // Move to limits
        for (float x = XDEF; x <= XMAX; x += 0.1) {
            write(x, YDEF);
            delay(10);
        }
        for (float y = YDEF; y <= YMAX; y += 0.1) {
            write(XMAX, y);
            delay(10);
        }
        for (float x = XMAX; x >= XMIN; x -= 0.1) {
            write(x, YMAX);
            delay(10);
        }
        for (float y = YMAX; y >= YMIN; y -= 0.1) {
            write(XMIN, y);
            delay(10);
        }
        for (float x = XMIN; x <= XDEF; x += 0.1) {
            write(x, YMIN);
            delay(10);
        }
        for (float y = YMIN; y <= YDEF; y += 0.1) {
            write(XDEF, y);
            delay(10);
        }
        // Move in a spiral
        float angle = 0;
        float radius = 3;
        while (radius < (XMAX - XMIN) / 2) {
            float x = XDEF + radius * cos(angle);
            float y = YDEF + radius * sin(angle);
            write(x, y);
            delay(7);
            angle += 0.1;
            if (angle > 2 * PI) {
                angle = 0;
                radius += 0.1;
            }
        }
    }


private:
    Servo tvcx, tvcy;

    float XMIN = 83;  // TVC X Min
    float XMAX = 105; // TVC X Max
    float YMIN = 73;  // TVC Y Min
    float YMAX = 95; // TVC Y Max
    float XDEF = 94;  // TVC X Default (zero position)
    float YDEF = 84;  // TVC Y Default (zero position)

    // --------- TVC Control --------- //

    float P = 1.2;
    float I = 0.1;
    float D = 0.1;
    float N = 50;

    float x_out;
    float y_out;

    bool FLIP_X = false;
    bool FLIP_Y = false;
    bool ROLL_COMP = true;

    Vec3D dir;
    boolean locked = false;

};