# Guidance-Navigation-Control-System

This project simulates 3D positional control using a combination of **PID controllers** and **Kalman filters** in a multithreaded C++ application. Each axis (X, Y, Z) is controlled independently to follow a target position while filtering noisy sensor data. The result is logged and visualized to demonstrate control accuracy.

---

## Features

- **Kalman Filtering** for state estimation from noisy position measurements
- **PID Control** for position correction on each axis
- **Multithreaded simulation** for concurrent axis control
- **CSV flight log** with position, velocity, control signals, and estimates
- **3D visualization** of true vs. estimated trajectories using Python

---

## Project Structure

<pre> Project Directory Structure . ├── src/ # C++ source files │ ├── KalmanFilter.cpp │ ├── KalmanFilter.h │ ├── PIDController.cpp │ ├── PIDController.h │ └── main.cpp ├── data/ # Output and input data │ └── flight_log_detailed.csv ├── scripts/ # Python visualization tools │ └── plot_trajectory.py ├── notebooks/ # Jupyter notebooks for analysis │ └── TrajectoryVisualization.ipynb ├── Makefile # Build instructions └── README.md # Project overview and instructions </pre>

## GNC Flowchart

![GNC Framework](https://github.com/user-attachments/assets/fd0a0579-ad80-4cac-a6a0-10bab72777f6)

---

## Results

X-axis visualization for PID controller/Kalman filter:
<img width="915" alt="Screenshot 2025-05-08 at 11 47 30 PM" src="https://github.com/user-attachments/assets/9136c710-239b-447d-9654-300cbd5b1e5e" />

3D visualization of UAV flight path:
<img width="916" alt="Screenshot 2025-05-08 at 11 48 05 PM" src="https://github.com/user-attachments/assets/1aaa7102-c362-49f5-8a60-4e4087901d7a" />

## Build and Run

### Requirements

- C++ compiler with C++11 or later
- Python 3.x (for plotting)
  - `matplotlib`, `pandas` (install via pip)

### Compile and Run

```bash
make
./main


