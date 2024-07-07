#include <Arduino.h>
#include <Serial.h>
#include <Servo.h>
#include <ArduPID.h>


class TVC {
public:
    void begin() {
        dir = Orientation(0, 0);
        tvcx.attach(5);
        tvcy.attach(6);

        pid_x.begin(&yaw, &x_out, 0, P, I, D);
        pid_x.setOutputLimits(XMIN, XMAX);
        tvcx.write(XDEF);

        pid_y.begin(&pitch, &y_out, 0, P, I, D);
        pid_y.setOutputLimits(YMIN, YMAX);
        tvcy.write(YDEF);
    }

    Orientation update(Orientation o) {
        if (!locked) {
            yaw = o.yaw;
            pitch = o.pitch;
            pid_x.compute();
            pid_y.compute();
            tvcx.write(x_out);
            tvcy.write(y_out);
            return Orientation(x_out, y_out);

        }
        else {
            tvcx.write(XDEF);
            tvcy.write(YDEF);

            return Orientation(XDEF, YDEF);
        }

    }

    /* Writes angles to TVC
    *  - angle range: -10 to +10 */
    void write(double x, double y) {
        tvcx.write(x + 90);
        tvcy.write(y + 90);
    }

    Orientation getAngle() {
        return Orientation(x_out, y_out);
    }

    void lock() {
        locked = true;
    }

    void unlock() {
        locked = false;
    }


private:
    Servo tvcx, tvcy;

    double yaw, pitch;

    // placeholder values; replace with actual limits
    const double XMIN = 80;  // TVC X Min
    const double XMAX = 100; // TVC X Max
    const double YMIN = 80;  // TVC Y Min
    const double YMAX = 100; // TVC Y Max
    const double XDEF = 90;  // TVC X Default (zero position)
    const double YDEF = 90;  // TVC Y Default (zero position)

    // --------- TVC Control --------- //
    ArduPID pid_x;
    ArduPID pid_y;
    // old PID values
    // P = 8.58679935818825;
    // I = 12.4428493210038;
    // D = 0.482664861486399;
    const double P = 1.2;
    const double I = 0.1;
    const double D = 0.1;

    double x_out = XDEF;
    double y_out = YDEF;

    Orientation dir;
    boolean locked = false;

};