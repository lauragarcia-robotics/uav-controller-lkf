# UAV controller + Linear Kalman Filter

C++ implementation of a position controller and a Linear Kalman Filter for a multirotor UAV in simulation. 

## What I implemented 
- PID position control for x/y/z with feedforward acceleration terms.
- Conversion to desired tilt commands and thrust.
- Linear Kalman Filter for 3D position estimation using a constant-acceleration model.
- Filter tuning via Q/R covariances and controller gains via parameter file.

## State estimation
State: 
x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T

Measurement: 
z = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T

## Repository content
This repo contains only my implementation files: 
- `src/controller.cpp`
- `src/lkf.cpp`
- `config/user_params.yaml`

## Tech stack
- C++, EIgen
- UAV simulation workflow (Gazebo/RViz)
