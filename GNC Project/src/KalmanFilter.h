#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    // State and covariance matrix
    double x; // State (estimate)
    double P; // Covariance

    // System parameters
    double A; // State transition matrix
    double B; // Control input matrix
    double H; // Measurement matrix
    double Q; // Process noise covariance
    double R; // Measurement noise covariance
    double K; // Kalman Gain

    // Constructor
    KalmanFilter(double init_x, double init_P, double A, double B, double H, double Q, double R);
    
    // Destructor
    ~KalmanFilter();

    // Prediction step
    void predict(double u);

    // Update step
    void update(double z);

    // Get the current state
    double getState();
};

#endif