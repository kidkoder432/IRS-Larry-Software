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
    }

    void begin(float _Kp, float _Ki, float _Kd, float _b, float _dt, float _min, float _max) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        bias = _b;
        dt = _dt;
        min = _min;
        max = _max;
    }

    float update(float target, float current, float _dt) {
        dt = _dt;
        alpha = N * dt;
        b = alpha / (alpha + 2);
        error = target - current;

        // Proportional
        p = Kp * error;

        // Integral
        i = Ki * integrated_error;

        // Derivative + low-pass filtering

        // y is filtered
        // x is raw

        // https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/
        // alpha = N*dt
        // a = -(alpha - 2) / (alpha + 2)
        // b = alpha / (alpha + 2)
        // y = a * last_y + b * (x + last_x)
        // a = -(alpha - 2) / (alpha + 2);
        // b = alpha / (alpha + 2);
        // filtered_error = a * last_filter + b * (error + last_error);

        // EMA Filter
        alpha = 1.0 / N;
        filtered_error = alpha * filtered_error + (1 - alpha) * error;

        // d = Kd * (filtered_error - last_filter) / dt;

        if (dt <= 0.02) {
            d = Kd * (filtered_error - last_filter) / dt;
        }
        else {
            d = 0;
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
    float a = 0;
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
