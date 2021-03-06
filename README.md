# CarND-Controls-MPC
This project is done as a part of the Nanodegree - Self-Driving Car Engineer provided by Udacity. The scope of this project is the implementation of **Model Predictive Control (MPC)**, which allows a car (in a simulator) to follow the given track by adjusting the steering angle, throttle, and brake. A latency of 100ms between actuator commands is part of the system.

![image_screen](https://github.com/gada1982/CarND-MPC-Project/blob/master/info_for_readme/Screenshot_jpg.jpg)

When a human is driving he or she tries to stay within the preferred lane, keeps the desired speed, and turns as smooth as possible. To get this job done we look ahead. As an example, the driver will slow down before driving into a sharp turn and will accelerate after reaching the middle of the curve. 

A car, which is using a Model Predictive Control, can imitate this behavior of taking future into account. By predicting different paths and choosing the best one, it can adapt it's driving behavior to different unknown situations. The behavior can be specified by choosing a car-specific cost-function, which defines constraints like how smooth the car should move or how far it is allowed to go off the planned path.

## Overview - Processing Steps
- **Get waypoints:** The MPC gets the referance path from the simulator as an array of waypoints (ptsx/ptsy), which are in map coordinate system. 
- **Get actual vehicle data:** The MPC gets actual vehicle data like the position and the orientation of the car or the actual speed and steering angle from the simlulator.
- **Transform coordinate system:** In the map coordinate system x is positive to the right and y is positive up (like in a standard graph). In car coordinate system x is positive in straight driving direction and y is positive 90 degree to the left. The waypoints have to be transformed to the vehicle's coordinate system, because the cross-track error (CTE) and the orientation error (EPSI) have to be calculated in this reference space. 
- **Fitting a polynomial:** The trajectory is approximated with a 3rd order polynomial.
- **System latency:** The whole system has a latency of 100ms, which has to be taken into account.
- **Calculate error:** Out of the polynomial coefficients and the estimated position of the car, after the latency, the cross-track error (CTE) and the orientation error (EPSI) are calculated.
- **Predict future states:** N states with N-1 changes of the actuators are predicted within a prediction horizon `T=N*dt` (N = number of timesteps, dt = time between two actuations). The states are in the vehicle's coordinate system.
- **Control car:** The first prediction is taken to control the cars steering and acceleration (throttle/brake).
- **Visualize:** Display the predicted trajectory path (where the car will be) in green, and the reference path (where the car should be) in yellow.
- **Repeat all steps**

## Defining the Model
Through the implemented MPC, the vehicle follows the reference path, provided by a simulator in the map coordinate system, by calculating and setting predicted actuator outputs for steering and acceleration (throttle/brake).

### Vehicle State
The actual state of the vehicle is defined by:
- **Position of the car:** car_px, car_py
- **Orientation of the car:** car_psi
- **Actual speed:** v_miles

For solving the optimization problem error values (distance and orientation) are needed. The cross-track error *(cte)* is the difference between the path, which the car should follow and the current vehicle position. This is coupled with the y-coordinate in the coordinate system of the vehicle. *Epsi* is the orientation error of the vehicle. 

These parameters are used the following way to solve the optimization problem:
```
Eigen::VectorXd state(6);
state << car_px, car_py, car_psi, v_miles, cte, epsi;

// Solve the model given the vehicle's state and polynomial coefficients
auto vars = mpc.Solve(state, coeffs);
```
### Vehicle's Actuators
The system tries to reach the target state by setting values for the actuators. The orientation can be influenced by the steering angle *(delta)* and the speed by the acceleration *(a)*.
```
// Normalize steer_value to [-1, 1]
double steer_value = -vars[delta_start] / deg2rad(25);
double throttle_value = vars[a_start];
```
### Kinematic Model
A kinematic bicycle model is used as vehicle model. This model simplifies the real world by ignoring tire forces, gravity, and mass. This reduces the accuracy of the model but makes it more tractable. For low and moderate speeds, this type of kinematic models delivers a useful approximation of the real vehicle dynamics. 

The kinematic model is implemented through the following equations:
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
```

The position of the vehicle is defined by *(x,y)*. *Psi* stands for its orientation and *v* for its velocity. *Delta* and *a* represent actuators (for steering and acceleration - throttle/brake). The distance between the front of the vehicle and its center of gravity (CoG) is defined by *Lf*. *dt* is the time span between the actual state and the following.
 
 ### Error
 The error (distance and orientation) is calculated with the following equations: 
 ```
 cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
 epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
 ```
### Cost
The MPC includes a cost-model that defines the dynamics of the system and determines how much parameters are taken into account. Finally the MPC optimizes by choosing the best fitting prediction to get cost down to 0. The model can be tuned by adapting the multiplication factors of the single parameters. The higher the single parts are the more important the parameter is for the final cost optimization.

The cost-model is defined as follows:
```
// The part of the cost based on the reference state (cte, epsi, v).
double mult_ref_cte_epsi = 2200.0;
double mult_ref_v = 1.0;
for (int t = 0; t < N; t++) {
  fg[0] += mult_ref_cte_epsi * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += mult_ref_cte_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += mult_ref_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the usage of the actuators (steering, throttle/brake)
// to prevent too hard steering and acceleration after strong changes in the vehicle's state
// (e.g.: cte / epsi high because of the need to change lane.)
double mult_change_rate = 10.0;
for (int t = 0; t < N - 1; t++) {
  fg[0] += mult_change_rate * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += mult_change_rate * CppAD::pow(vars[a_start + t], 2);
}

// Minimize the difference between the next actuator state and the current one
// The next control input should be similar to the current one to avoid erratic driving.
double mult_gap_action_delta = 300.0;
double mult_gap_action_a = 20.0;
for (int t = 0; t < N - 2; t++) {
  fg[0] += mult_gap_action_delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += mult_gap_action_a * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```
## Timestep Length and Elapsed Duration (N & dt)
The prediction horizon *(T)* is the duration over which future predictions are made.`T=N*dt` (N = number of timesteps in the horizon, dt = time between two actuations). Parameters are set be defining a suitable value for T first. This depends how fast the environment is changing and how likely it is the future states can be predicted in a useful way. The higher N is the more computing power is necessary. Smaller values of dt result in higher frequent actuations, which makes it easier to accurately approximate a continuous reference trajectory.

The values have been set to the following numbers after lots of experiementation:
```
// Define the prediction horizon T, which is calculated by N*dt
size_t N = 10;
double dt = 0.12;
```
Values for *N* between *5 - 20* and for *dt* between *0.05 - 0.2* have been tested in various combinations.

## Target Speed
In order to test the quality of the control system, the target speed was gradually increased. The main requirement was that the vehicle has to master the route safely. 70 miles/h can be managed on a mid-class computing system. Because of the non-real-time behavior of the simulator and its connection to the control system, this can vary when using other systems. In a real automotive application, this task is managed by a strictly deterministic system to minimize timing effects.

## Polynomial Fitting
After transforming the waypoints into the car's coordinate system a polynomial third-order is fitted to these points. A polynomial of this kind is chosen because they can approximate most roads and are not too complex to process.
```
// Fit a polynomial 
auto coeffs = polyfit(car_x, car_y, 3);
```
## Model Predictive Control with Latency
The system operates with a latency of 100ms. This is introduced to get a more realistic simulation of real-world driving, where inputs of sensors and the result of using actuators have a time shift. This can be explained through processing time, signal runtime, mechanical properties and other variables, and so the system won't react instantaneously.

Latency is taken into account by setting the initial value of the state vector. This is done by taking the current state taken from the simulator and projected them forward one latency time step.
```
// Handle system latency of 100ms
// Predict state after latency
double dt_lat = 0.1;
double car_px = v_m*dt_lat;
const double Lf = 2.67;
double car_psi = -v_m*steering_angle*dt_lat/Lf;
```
## Unit Conversion
The simulator gives the actual speed in *miles/h*, but distance is measured in *m* and time spans are measured in *s*. This has to be considered by:
```
// Convert actual speed from miles/h to m/s
double v_m = v_miles * 0.44704;
```

The systems steering is between *[-25/25] degree*, which is *[-0.436332/-0.436332] radians* but the signal, which has to be sent to the simulator is a procentual value between *[-1/1]*. This has to be considered by:
```
// Normalize steer_value to [-1, 1]
double steer_value = -vars[delta_start] / deg2rad(25);
```
## Simulation
The following [video](https://youtu.be/ozLKdjQXu7Q) shows a car, controlled by the implemented MPC controller, successfully driving on a given track with max. speed of 70 miles/h. The other [video](https://youtu.be/D0RX-5Z9zP4) shows a car, which uses the same MPC controller, at a higher reference speed of more than 100 miles/h.

## Reflection
The project was very interesting and challenging but the result was convincing. The advances through the usage of MPC against PID control are immense. In contrast to the PID control, which always requires a compromise in tuning and more or less strongly oscillates around the target value, MPC works much better on both straights and curves.

Future work will be invested to improve parameter settings and manage to drive with higher speed.

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
