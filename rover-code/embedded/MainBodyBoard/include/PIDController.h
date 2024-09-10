#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


#define OUTPUT_MIN 0
#define OUTPUT_MAX 500

class PIDController {
public:
    // Constructor
    PIDController(double kp, double ki, double kd);

    // Update method
    double update(double targeted_velocity, double current_velocity);

private:
    double kp;
    double ki;
    double kd;
    double integral;
    double previous_error;
};

#endif // PID_CONTROLLER_H
