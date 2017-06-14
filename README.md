# CarND-Controls-MPC
This project is done as a part of the Nanodegree - Self-Driving Car Engineer provided by Udacity. The scope of this project is the implementation of Model Predictive Control (MPC), which allows a car (in a simulator) to follow the given track by adjusting the steering angle, throttle, and brake. A latency of 100ms between actuator commands is part of the system.

[!image_screen](https://github.com/gada1982/CarND-MPC-Project/blob/master/info_for_readme/Screenshot_jpg.jpg)

## Implementation
### The Model
Through the implemented MPC, the vehicle follows the trajectory path provided by a simulator in the map coordinate system by calculating and setting predicted actuator outputs for steering and acceleration (throttle/brake).

A kinematic bicycle model is used as vehicle model. This model simplifies the real world by ignoring tire forces, gravity, and mass. This reduces the accuracy of the model but makes it more tractable. For low and moderate speeds, this type of kinematic models delivers a useful approximation of the actual vehicle dynamics. 

The kinematic model is implemented through the following equations:

`x[t+1] = x[t] + v[t] * cos(psi[t]) * dt`

`y[t+1] = y[t] + v[t] * sin(psi[t]) * dt`

`psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt`

`v[t+1] = v[t] + a[t] * dt`

The position of the vehicle is defined by (x,y). Psi stands for its orientation and v for its velocity. delta and a represent actuators (e.g.: for steering and aceleration - throttle/brake). The distance between the front of the vehicle and its center of gravity (CoG) is defined by Lf.
 
 The error (distance and orientation) is calculated with the following equations:
 
 `cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`
 
 `epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`
 
The cross-track error (cte) is the difference between the path which the car should follow and the current vehicle position. This is coupled with the y-coordinate in the coordinate system of the vehicle. Epsi is the orientation error of the vehicle. 

### TODO
- The MPC gets the trajectory path from the simulator as an array of waypoints (ptsx/ptsy), which are in map coordinate system. 

- The waypoints have to be transformed to the vehicle's coordinate system, because the cross-track error (CTE) and the orientation error (EPSI) have to be calculated in this reference space. 

- The trajectory is approximated with a 3rd order polynomial.

- Out of the polynomial coefficients and the estimated position of the car (for details see ????) the cross-track error (CTE) and the orientation error (EPSI) are calculated.

- N states with N-1 changes of the actuators are predicted within a prediction horizon T=N*dt (N = number of timesteps, dt = time between two actuation). The states are in the vehicle's coordinate system.

### Timestep Length and Elapsed Duration (N & dt)

### Polynomial Fitting and MPC Preprocessing

### Model Predictive Control with Latency


## Simulation

The following [video](https://youtu.be/7zCltY4EiUc) shows a car, controlled by the implemented MPC controller, successfully driving on a given track.

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
