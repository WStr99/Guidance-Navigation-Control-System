
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double minOutput = -1e9, double maxOutput = 1e9);

    void reset();
    double compute(double setpoint, double measurement, double dt);

private:
    double kp_, ki_, kd_;
    double minOutput_, maxOutput_;
    double prevError_ = 0.0;
    double integral_ = 0.0;
};