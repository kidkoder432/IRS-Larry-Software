#include <math.h>

class PID {
public:

    float p;
    float i;
    float d;

    void update_gains(float _Kp, float _Ki, float _Kd, float _FilterN) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        N = _FilterN;

        alpha = N * dt;
        b = alpha / (alpha + 2);
    }

    void begin(float _Kp, float _Ki, float _Kd, float _b, float _dt, float _min, float _max) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        bias = _b;
        dt = _dt;
        min = _min;
        max = _max;

        alpha = N * dt;
        b = alpha / (alpha + 2);
    }

    float update(float target, float current, float _dt) {
        dt = _dt;
        error = target - current;

        // Proportional
        p = Kp * error;

        // Integral
        i = Ki * integrated_error;

        // Derivative + low-pass filtering

        if (N > 0) {
            filtered_error = b * (error + last_error) + (1 - b * 2) * last_filter;
            d = Kd * (filtered_error - last_filter) / dt;
        }
        else {
            d = Kd * (error - last_error) / dt;
        }
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

        return p + i + d;
        // // Output saturation
        // return clip(p + i + d + bias, min, max);
    }

    void reset() {
        integrated_error = 0;
        last_filter = 0;
        last_error = 0;
        filtered_error = 0;
        error = 0;
    }
private:

    float Kp;
    float Ki;
    float Kd;
    float N;

    float bias;
    float dt;
    float min;
    float max;
    float integrated_error = 0;
    float error;
    float last_error = 0;
    float filtered_error = 0;
    float last_filter = 0;


    float alpha = 0;
    float b = 0;
    float clip(float value, float min, float max) {
        return min < value && value < max ? value : min < value ? max : min;
    }

    bool doIntegratorClamp(float out, float b) {
        bool saturated = out < min || out > max;
        bool sameSign = sign(out - b) == sign(error);

        return saturated && sameSign;
    }
};
