#ifndef __PID_H__
#define __PID_H__

class PID {
private:
    double Kp;
    double Ki;
    double Kd;
    double prev_error;
    double integral;
    double target;
    double windup;
    double last_output;
    double max_slew;

    double clamp(double n, double smallest, double largest);
    double slew(double output, double dt);

public:
    PID(double Kp, double Ki, double Kd, double target, double windup, double max_slew);

    double update(double input, double dt);
    void setTarget(double new_target);
    double getTarget() const;
    void reset();
};

#endif // __PID_H__
