#include <math.h>

class PID {
public:

    double Kp;
    double Ki;
    double Kd;

    double p;
    double i;
    double d;

    double N;
    void begin(double _Kp, double _Ki, double _Kd, double _b, double _dt, double _min, double _max) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        bias = _b;
        dt = _dt;
        min = _min;
        max = _max;
    }

    double update(double target, double current, double _dt) {
        dt = _dt;
        alpha = N * dt;
        b = alpha / (alpha + 2);
        error = target - current;
        
        // Proportional
        p = Kp * error;

        // Integral
        i = Ki * integrated_error;

        // Derivative + low-pass filtering
        filtered_error = b * (error + last_error) + (1 - b * 2) * last_filter;
        d = Kd * (filtered_error - last_filter) / dt;

        // Integrator clamping
        if (doIntegratorClamp(p + i + d + bias, bias)) {
            integrated_error += 0;
        }
        else {
            integrated_error += last_error * dt;
        }

        // Set previous state
        last_filter = filtered_error;
        last_error = error;

        // Output saturation
        return clip(p + i + d + bias, min, max);
    }

    void reset() {
        integrated_error = 0;
        last_filter = 0;
        last_error = 0;
        filtered_error = 0;
        error = 0;
    }
private:

    double bias;
    double dt;
    double min;
    double max;
    double integrated_error = 0;
    double error;
    double last_error = 0;
    double filtered_error = 0;
    double last_filter = 0;


    double alpha = 0;
    double b = 0;
    double clip(double value, double min, double max) {
        return min < value && value < max ? value : min < value ? max : min;
    }

    bool doIntegratorClamp(double out, double b) {
        bool saturated = out < min || out > max;
        bool sameSign = sign(out - b) == sign(error);  

        return saturated && sameSign;
    }
};
