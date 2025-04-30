#include "PIDController.h"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double minOutput, double maxOutput)
    : kp_(kp), ki_(ki), kd_(kd), minOutput_(minOutput), maxOutput_(maxOutput) {}

void PIDController::reset() {
    prevError_ = 0.0;
    integral_ = 0.0;
}

double PIDController::compute(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;
    const double integralLimit = 10.0; // or whatever makes sense for your system
    integral_ += error * dt;
    if (integral_ > integralLimit) integral_ = integralLimit;
    if (integral_ < -integralLimit) integral_ = -integralLimit;
    double derivative = (error - prevError_) / dt;
    prevError_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, minOutput_, maxOutput_);
}