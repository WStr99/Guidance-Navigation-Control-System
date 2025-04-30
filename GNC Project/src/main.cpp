#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <random>
#include <fstream>
#include "KalmanFilter.h"
#include "PIDController.h"

std::mutex stateMutex;

struct AxisState {
    double pos = 0.0;
    double vel = 0.0;
    double estimate = 0.0;
    double control = 0.0;
    double sensor = 0.0;
};

struct SensorData {
    double value;
    double control;
};

SensorData simulateSensor(double trueValue, double noise_std = 0.1, double control = 0.0) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, noise_std);
    double noisyValue = trueValue + distribution(generator);
    return { noisyValue, control };
}

void controlLoopAxis(
    KalmanFilter kf,
    PIDController pid,
    AxisState& state,
    double target,
    double externalForce,
    double drag,
    double gravity,
    int totalSteps,
    double dt
) {
    for (int i = 0; i < totalSteps; ++i) {
        SensorData sensor = simulateSensor(state.pos, 0.3, state.vel);

        kf.predict(state.vel);
        kf.update(sensor.value);
        double estimate = kf.getState();

        double control = pid.compute(target, estimate, dt);
        double netForce = control + externalForce - drag * state.vel - gravity;
        double acc = netForce;

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            state.sensor = sensor.value;
            state.estimate = estimate;
            state.control = control;
            state.vel += acc * dt;
            state.pos += state.vel * dt;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
}

int main() {
    const double dt = 0.1;
    const int steps = 150;

    AxisState x, y, z;

    KalmanFilter kf_x(0.0, 1.0, 1.0, 0.2, 1.0, 0.1, 0.3);
    KalmanFilter kf_y(0.0, 1.0, 1.0, 0.2, 1.0, 0.1, 0.3);
    KalmanFilter kf_z(0.0, 1.0, 1.0, 0.2, 1.0, 0.1, 0.3);

    PIDController pid_x(2.5, 0.1, 2.0, -30, 30);
    PIDController pid_y(2.5, 0.1, 2.0, -30, 30);
    PIDController pid_z(2.5, 0.1, 2.0, -30, 30);

    std::ofstream logFile("flight_log_detailed.csv");
    logFile << "Time,"
            << "xEstimate,yEstimate,zEstimate,"
            << "xTrue,yTrue,zTrue,"
            << "xSensor,ySensor,zSensor,"
            << "vx,vy,vz,"
            << "ux,uy,uz\n";

    std::thread threadX(controlLoopAxis, kf_x, pid_x, std::ref(x), 100.0, 0.0, 0.1, 0.0, steps, dt);
    std::thread threadY(controlLoopAxis, kf_y, pid_y, std::ref(y), 75.0, 0.0, 0.1, 0.0, steps, dt);
    std::thread threadZ(controlLoopAxis, kf_z, pid_z, std::ref(z), 50.0, 0.0, 0.1, 9.81, steps, dt);

    for (int i = 0; i < steps; ++i) {
        double time = i * dt;

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            logFile << time << ","
                    << x.estimate << "," << y.estimate << "," << z.estimate << ","
                    << x.pos << "," << y.pos << "," << z.pos << ","
                    << x.sensor << "," << y.sensor << "," << z.sensor << ","
                    << x.vel << "," << y.vel << "," << z.vel << ","
                    << x.control << "," << y.control << "," << z.control << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    threadX.join();
    threadY.join();
    threadZ.join();
    logFile.close();

    return 0;
}