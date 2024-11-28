#include <Arduino.h>
#include <pid.h>
#include <config.h>

#define ACTUAL_PID 1
#define PRINT_OUTPUT 1

#define FLIP_X 0
#define FLIP_Y 0

double clamp(double x, double min, double max) { return (x < min) ? min : (x > max) ? max : x; }

class TVC {
public:

    PID pid_x;
    PID pid_y;

    void updatePID(PID& pid, double _Kp, double _Ki, double _Kd, double filterN) {
        pid.Kp = _Kp;
        pid.Ki = _Ki;
        pid.Kd = _Kd;
        pid.N = filterN;
    }

    void updateBounds(Config2& config) {
        XMIN = config["XMIN"];
        XMAX = config["XMAX"];

        YMIN = config["YMIN"];
        YMAX = config["YMAX"];

        XDEF = config["XDEF"];
        YDEF = config["YDEF"];

    }

    void begin() {
        dir = Vec3D(0, 0, 0);
        tvcx.attach(5);
        tvcy.attach(6);

        pid_x.begin(P, I, D, XDEF, DELTA_TIME, XMIN, XMAX);
        tvcx.write(XDEF);

        pid_y.begin(P, I, D, YDEF, DELTA_TIME, YMIN, YMAX);
        tvcy.write(YDEF);
    }

    // X axis = yaw correction, Y axis = pitch correctin
    Vec2D update(Vec3D o, double dt) {
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

            double tvcXRaw = (x_out - XDEF) * PI / 180;
            double tvcYRaw = (y_out - YDEF) * PI / 180;

            double cr = cos(dir.x * PI / 180);
            double sr = sin(dir.x * PI / 180);

            x_out = tvcXRaw * cr - tvcYRaw * sr;
            y_out = tvcXRaw * sr + tvcYRaw * cr;

            x_out = (x_out * 180 / PI) + XDEF;
            y_out = (y_out * 180 / PI) + YDEF;
            #if FLIP_X
                x_out = -x_out; 
            #endif 
            #if FLIP_Y
                y_out = -y_out; 
            #endif
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
    void write(double x, double y) {
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
        if (abs(dir.y) > 30 || abs(dir.x) > 30) {
            tvcx.write(XDEF);
            tvcy.write(YDEF);
            locked = true;
        }
    }


private:
    Servo tvcx, tvcy;

    // placeholder values; replace with actual limits
    double XMIN = 83;  // TVC X Min
    double XMAX = 105; // TVC X Max
    double YMIN = 73;  // TVC Y Min
    double YMAX = 95; // TVC Y Max
    double XDEF = 94;  // TVC X Default (zero position)
    double YDEF = 84;  // TVC Y Default (zero position)

    // --------- TVC Control --------- //

    // old PID values
    // P = 8.58679935818825;
    // I = 12.4428493210038;
    // D = 0.482664861486399;
    const double P = 1.2;
    const double I = 0.1;
    const double D = 0.1;

    double x_out;
    double y_out;

    Vec3D dir;
    boolean locked = false;

};