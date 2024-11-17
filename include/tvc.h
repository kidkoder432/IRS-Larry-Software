#include <Arduino.h>
#include <pid.h>

#define ROLL_TEST 1
#define PRINT_OUTPUT 1

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
            // Serial.println(dir.pitch);
            // Serial.println(dir.yaw);
            #if ROLL_TEST
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
            Quaternion tvcQuat(0, 0, (y_out - YDEF) * PI / 180, (x_out - XDEF) * PI / 180);
            Quaternion rollQuat = Quaternion::from_axis_angle(-dir.x * PI / 180, 1, 0, 0);

            Quaternion correctedQuat = (rollQuat * tvcQuat) * rollQuat.conj();
            x_out = correctedQuat.d * 180 / PI + XDEF;
            y_out = correctedQuat.c * 180 / PI + YDEF;

            #if PRINT_OUTPUT
            Serial.print("X/Y Adjust: ");
            Serial.print(x_out);
            Serial.print(" ");
            Serial.println(y_out);
            #endif

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
    const double XMIN = 83;  // TVC X Min
    const double XMAX = 105; // TVC X Max
    const double YMIN = 73;  // TVC Y Min
    const double YMAX = 95; // TVC Y Max
    const double XDEF = 94;  // TVC X Default (zero position)
    const double YDEF = 84;  // TVC Y Default (zero position)

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