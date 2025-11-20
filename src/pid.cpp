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
    
        double clamp(double n, double smallest, double largest) {
            if (n < smallest) return smallest;
            if (n > largest) return largest;
            return n;
        }
    
        double slew(double output, double dt) {
            double max_delta = max_slew * dt;
            double delta = output - last_output;
            delta = clamp(delta, -max_delta, max_delta);
            last_output += delta;
            return last_output;
        }
    
    public:
        PID(double Kp, double Ki, double Kd, double target, double windup, double max_slew)
            : Kp(Kp), Ki(Ki), Kd(Kd), target(target), windup(windup), max_slew(max_slew),
              prev_error(0.0), integral(0.0), last_output(0.0) {}
    
        double update(double input, double dt) {
            double error = target - input;
            integral += error * dt;
            integral = clamp(integral, -windup, windup);
            double derivative = (error - prev_error) / dt;
            prev_error = error;
    
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

    