#include <Arduino.h>
#include <pid.h>
#include <Servo.h>
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
    }

    void begin(float deltaTime) {
        dir = Vec3D(0, 0, 0);
        tvcx.attach(5);
        tvcy.attach(6);

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

            float tvcXRaw = (x_out - XDEF) * PI / 180;
            float tvcYRaw = (y_out - YDEF) * PI / 180;

            float cr = cos(dir.x * PI / 180);
            float sr = sin(dir.x * PI / 180);

            x_out = tvcXRaw * cr - tvcYRaw * sr;
            y_out = tvcXRaw * sr + tvcYRaw * cr;

            x_out = (x_out * 180 / PI) + XDEF;
            y_out = (y_out * 180 / PI) + YDEF;

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
        tvcx.write(x);
        tvcy.write(y);
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


private:
    Servo tvcx, tvcy;

    // placeholder values; replace with actual limits
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

    Vec3D dir;
    boolean locked = false;

};