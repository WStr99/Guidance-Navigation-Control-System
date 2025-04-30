#include <iostream>
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double init_x, double init_P, double A, double B, double H, double Q, double R) {
    this->x = init_x;
    this->P = init_P;
    this->A = A;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::predict(double u) {
    x = A * x + B * u;
    P = A * P * A + Q;
}

void KalmanFilter::update(double z) {
    K = P * H / (H * P * H + R);
    x = x + K * (z - H * x);
    P = (1 - K * H) * P;
}

double KalmanFilter::getState() {
    return x;
}
