#include <math.h>

class PID {
public:

    double Kp;
    double Ki;
    double Kd;
    double cutoff;
    void begin(double _Kp, double _Ki, double _Kd, double _b, double _dt, double _min, double _max) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        b = _b;
        dt = _dt;
        min = _min;
        max = _max;
    }

    double update(double target, double current, double _dt) {
        dt = _dt;
        alpha = dt / (dt + 1/cutoff);
        error = target - current;
        // Proportional
        double p = Kp * error;

        // Integral
        double i = Ki * integrated_error;

        // Derivative + low-pass filtering
        double filtered_error = (1-alpha) * (last_filter) + alpha * error;
        double d = Kd * (filtered_error - last_filter) / dt;

        // Integrator clamping
        if (doIntegratorClamp(p + i + d + b, b)) {
            integrated_error += 0;
        }
        else {
            integrated_error += last_error * dt;
        }

        // Set previous state
        last_filter = filtered_error;
        last_error = error;

        // Output saturation
        return clip(p + i + d + b, min, max);
    }

    double P() {
        return Kp * error;
    }

    double I() {
        return Ki * integrated_error;
    }

    double D() {

        return Kd * (error - last_error) / dt;
    }
private:

    double b;
    double dt;
    double min;
    double max;
    double integrated_error = 0;
    double error;
    double last_error = 0;
    double last_filter = 0;
    double alpha;
    double clip(double value, double min, double max) {
        return min < value && value < max ? value : min < value ? max : min;
    }

    bool doIntegratorClamp(double out, double b) {
        bool saturated = out < min || out > max;
        bool sameSign = sign(out - b) == sign(error);  

        return saturated && sameSign;
    }
};
