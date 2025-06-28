#include <Arduino.h>
#include <pid.h>
#include <config.h>

#define ACTUAL_PID 1
#define PRINT_OUTPUT 0

float clamp(float x, float min, float max) { return (x < min) ? min : (x > max) ? max : x; }

class TVC {
public:

    PID pid_x;
    PID pid_y;

    void setup(float deltaTime, Config& config) {
        dir = Vec3D(0, 0, 0);
        tvcx.attach(TVC_X_PIN);
        tvcy.attach(TVC_Y_PIN);

        P = config["Kp"] ? config["Kp"] : P; 
        I = config["Ki"] ? config["Ki"] : I;
        D = config["Kd"] ? config["Kd"] : D;
        N = config["N"] ? config["N"] : N;

        XMIN = config["XMIN"] ? config["XMIN"] : XMIN;
        XMAX = config["XMAX"] ? config["XMAX"] : XMAX;
        YMIN = config["YMIN"] ? config["YMIN"] : YMIN;
        YMAX = config["YMAX"] ? config["YMAX"] : YMAX;

        XDEF = config["XDEF"] ? config["XDEF"] : XDEF;
        YDEF = config["YDEF"] ? config["YDEF"] : YDEF;

        pid_x.begin(P, I, D, XDEF, deltaTime, XMIN, XMAX);
        pid_y.begin(P, I, D, YDEF, deltaTime, YMIN, YMAX);

        FLIP_X = config["FLIP_X"] > 0;
        FLIP_Y = config["FLIP_Y"] > 0;
        ROLL_COMP = config["ROLL_COMP"] > 0;

        locked = true;
        reset();

    }

    // X axis = yaw correction, Y axis = pitch correctin
    Vec2D update(Vec3D o, float dt) {

        // Hack for when we just finished calibration and dt is absurd
        if (dt > 0.2) {
            dt = 0.015;
        }

        if (!locked) {
            dir = o;
            x_out = pid_x.update(0, dir.z, dt);
            y_out = pid_y.update(0, dir.x, dt);

        #if PRINT_OUTPUT 
            Serial.print("X/Y Raw: ");
            Serial.print(x_out);
            Serial.print(" ");
            Serial.print(y_out);
            Serial.print("\t");
        #endif
            // if (ROLL_COMP) {
            //     float tvcXRaw = (x_out)*PI / 180;
            //     float tvcYRaw = (y_out)*PI / 180;

            //     float cr = cos(dir.y * PI / 180);
            //     float sr = sin(dir.y * PI / 180);

            //     x_out = tvcXRaw * cr + tvcYRaw * sr;
            //     y_out = tvcYRaw * cr - tvcXRaw * sr;

            //     x_out = (x_out * 180 / PI);
            //     y_out = (y_out * 180 / PI);
            // }

            if (FLIP_X) x_out = -x_out;
            if (FLIP_Y) y_out = -y_out;


        #if PRINT_OUTPUT

            Serial.print("X/Y Adjust: ");
            Serial.print(x_out);
            Serial.print(" ");
            Serial.println(y_out);
        #endif

            // // Scale values less than defaults so that they have the full range of motion
            // if (x_out < 0) {
            //     x_out *= (XDEF - XMIN) / (XMAX - XDEF);
            // }

            // if (y_out < 0) {
            //     y_out *= (YDEF - YMIN) / (YMAX - YDEF);
            // }

            x_out += XDEF;
            y_out += YDEF;

            x_out = clamp(x_out, XMIN, XMAX);
            y_out = clamp(y_out, YMIN, YMAX);

            move(x_out, y_out);
            return Vec2D(x_out, y_out);

        }
        else {
            move(x_out, y_out);
            return Vec2D(XDEF, YDEF);
        }

    }

    // Writes angles to TVC
    void move(float x, float y) {
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
        move(XDEF, YDEF);
    }

    void unlock() {
        locked = false;
        reset();
    }

    void abort() {
        lock();
    }

    void reset() {
        move(XDEF, YDEF);
        pid_x.reset();
        pid_y.reset();
    }

    void nudge(int dir) {
        switch (dir) {
            case 0:
                x_out += 1;
                move(x_out, y_out);
                break;
            case 1:
                x_out -= 1;
                move(x_out, y_out);
                break;
            case 2:
                y_out += 1;
                move(x_out, y_out);
                break;
            case 3:
                y_out -= 1;
                move(x_out, y_out);
                break;
        }
    }

    void testRoutine() {
        // Move to limits

        move(XDEF, YDEF);
        Serial.println("Starting TVC test routine...");
        Serial.println("Moving to XMAX...");
        for (float x = XDEF; x <= XMAX; x += 0.1) {
            move(x, YDEF);
            delay(10);
        }
        Serial.println("Moving to YMAX...");
        for (float y = YDEF; y <= YMAX; y += 0.1) {
            move(XMAX, y);
            delay(10);
        }
        Serial.println("Moving to XMIN...");
        for (float x = XMAX; x >= XMIN; x -= 0.1) {
            move(x, YMAX);
            delay(10);
        }
        Serial.println("Moving to YMIN...");
        for (float y = YMAX; y >= YMIN; y -= 0.1) {
            move(XMIN, y);
            delay(10);
        }
        Serial.println("Moving to XDEF...");
        for (float x = XMIN; x <= XDEF; x += 0.1) {
            move(x, YMIN);
            delay(10);
        }
        Serial.println("Moving to YDEF...");
        for (float y = YMIN; y <= YDEF; y += 0.1) {
            move(XDEF, y);
            delay(10);
        }
        Serial.println("Returning to default position...");
        move(XDEF, YDEF);
        Serial.println("Moving in a spiral...");
        // Move in a spiral
        float angle = 0;
        float radius = 3;
        while (radius < (XMAX - XMIN) / 2) {
            float x = XDEF + radius * cos(angle);
            float y = YDEF + radius * sin(angle);
            move(x, y);
            delay(10);
            angle += 0.1;
            if (angle > 2 * PI) {
                angle = 0;
                radius += 1;
                Serial.print("Radius: ");
                Serial.println(radius);
            }
        }
    }


private:
    Servo tvcx, tvcy;

    float XMIN = 66;  // TVC X Min
    float XMAX = 114; // TVC X Max
    float YMIN = 73;  // TVC Y Min
    float YMAX = 121; // TVC Y Max
    float XDEF = 90;  // TVC X Default (zero position)
    float YDEF = 97;  // TVC Y Default (zero position)

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