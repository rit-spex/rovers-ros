#include "../../include/PIDController.h"

// Constructor
PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0) {}

// Update method
double PIDController::update(double targeted_velocity, double current_velocity) {
    double error = targeted_velocity - current_velocity;
    integral += error;
    double derivative = error - previous_error;
    double output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    return output;
}
