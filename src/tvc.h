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

        pid_x.begin(&dir.pitch, &x_out, 0, P, I, D);
        pid_x.setOutputLimits(XMIN, XMAX);
        tvcx.write(XDEF);

        pid_y.begin(&dir.pitch, &y_out, 0, P, I, D);
        pid_y.setOutputLimits(YMIN, YMAX);
        tvcy.write(YDEF);
    }

    void update(Orientation o) {
        if (!locked) {
            dir = o;
            pid_x.compute();
            pid_y.compute();
            tvcx.write(x_out);
            tvcy.write(y_out);
        }
        else {
            tvcx.write(XDEF);
            tvcy.write(YDEF);
        }
    }

    /* Writes angles to TVC 
    *  - angle range: -10 to +10 */
    void write(double x, double y) {
        tvcx.write(x + 90);
        tvcy.write(y + 90);
    }

    void lock() {
        locked = true;
    }

    void unlock() {
        locked = false;
    }


private:
    Servo tvcx, tvcy;

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
    const double P = 8.58679935818825;
    const double I = 12.4428493210038;
    const double D = 0.482664861486399;

    double x_out;
    double y_out;

    Orientation dir;
    boolean locked = false;

};