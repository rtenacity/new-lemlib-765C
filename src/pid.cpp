class PID {
    private:
        double Kp; // proportional gain
        double Ki; // integral gain
        double Kd; // derivative gain
        double prev_error;
        double integral; // integral
        double target; // target setpoint
        double windup; // windup limit
        double last_output;
        double max_slew; // max change rate
    
        double clamp(double n, double smallest, double largest) {
            if (n < smallest) return smallest; // clamp to smallest
            if (n > largest) return largest; // clamp to largest
            return n;
        }
    
        double slew(double output, double dt) {
            double max_delta = max_slew * dt; // maximum change allowed
            double delta = output - last_output; // desired change
            delta = clamp(delta, -max_delta, max_delta); // clamp change to max delta
            last_output += delta; // update last output
            return last_output; // return slewed output
        }
    
    public:
        PID(double Kp, double Ki, double Kd, double target, double windup, double max_slew)
            : Kp(Kp), Ki(Ki), Kd(Kd), target(target), windup(windup), max_slew(max_slew),
              prev_error(0.0), integral(0.0), last_output(0.0) {} // constructor
    
        double update(double input, double dt) {
            double error = target - input; // calculate error
            integral += error * dt; // update integral
            integral = clamp(integral, -windup, windup); // anti-windup
            double derivative = (error - prev_error) / dt; // calculate derivative
            prev_error = error; // update previous error
    
            double proportional = Kp * error;
            double integral_term = Ki * integral;
            double derivative_term = Kd * derivative;
    
            double output = proportional + integral_term + derivative_term;
            if (max_slew != 0.0) {
                return slew(output, dt);
            } else {
                return output;
            }
    
        
        }

        void setTarget(double new_target) { target = new_target; }
        double getTarget() const { return target; }
        void reset() {
            prev_error = 0.0;
            integral = 0.0;
            last_output = 0.0;
        }
    };

    