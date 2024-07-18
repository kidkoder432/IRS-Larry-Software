#include <math.h>

class PID
{
    public:
        void begin(double _Kp, double _Ki, double _Kd, double _b, double _dt, double _min, double _max)
        {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
            b = _b;
            dt = _dt;
            min = _min;
            max = _max;
        }

        double update(double target, double current, double _dt)
        {
            dt = _dt;
            error = target - current;
            double p = Kp * error;
            double i = Ki * integrated_error;
            double d = Kd * (error - last_error) / dt;

            if (doIntegratorClamp(p + i + d + b, b)) {
                integrated_error += 0;
                
            }
            else {
                integrated_error += error * dt; 
            }
            last_error = error;

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
        double Kp;
        double Ki;
        double Kd;
        double b;
        double dt;
        double min;
        double max;
        double integrated_error = 0;
        double error;
        double last_error = 0;
        double clip(double value, double min, double max) {
            return min < value && value < max ? value : min < value ? max : min;
        }

        bool doIntegratorClamp(double out, double b) {
            bool saturated = out < min || out > max;
            bool sameSign = sign(out - b) == sign(error);  //TODO: are these signs right?

            return saturated && sameSign;
        }
};
