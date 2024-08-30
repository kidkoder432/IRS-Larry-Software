#include <Arduino.h>
#include <Serial.h>
#include <Servo.h>
#include <pid.h>


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
        dir = Vec2D(0, 0);
        tvcx.attach(5);
        tvcy.attach(6);

        pid_x.begin(P, I, D, XDEF, DELTA_TIME, XMIN, XMAX);
        tvcx.write(XDEF);

        pid_y.begin(P, I, D, YDEF, DELTA_TIME, YMIN, YMAX);
        tvcy.write(YDEF);
    }

    Vec2D update(Vec2D o, double dt) {
        if (!locked) {
            dir = o;
            // Serial.println(dir.pitch);
            // Serial.println(dir.yaw);
            x_out = pid_x.update(0, dir.x, dt);
            y_out = pid_y.update(0, dir.y, dt);
            // Serial.print("X: ");
            // Serial.println(x_out);
            // Serial.print("Y: ");
            // Serial.println(y_out);
            tvcx.write(x_out);
            tvcy.write(y_out);

            return Vec2D(x_out, y_out);

        }
        else {
            // Serial.print("X: ");
            // Serial.println(x_out);
            // Serial.print("Y: ");
            // Serial.println(y_out);
            x_out = XDEF;
            y_out = YDEF;
            tvcx.write(x_out);
            tvcy.write(y_out);
            pid_x.reset();
            pid_y.reset();


            return Vec2D(XDEF, YDEF);
        }

    }

    /* Writes angles to TVC
    *  - angle range: -10 to +10 */
    void write(double x, double y) {
        tvcx.write(x + 90);
        tvcy.write(y + 90);
    }

    Vec2D getAngle() {
        return Vec2D(x_out, y_out);
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
    const double XMAX = 107; // TVC X Max
    const double YMIN = 73;  // TVC Y Min
    const double YMAX = 97; // TVC Y Max
    const double XDEF = 95;  // TVC X Default (zero position)
    const double YDEF = 85;  // TVC Y Default (zero position)

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

    Vec2D dir;
    boolean locked = false;

};