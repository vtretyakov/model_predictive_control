# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# [Rubric](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile.

The code compiles without errors with cmake and make as described in Basic Build Instructions.  `CMakeLists.txt` was not modified.

## Implementation

### The Model

The model used in the project is a simple global kinematic model that returns the next state vector, given a current state input vector and an actuator vector. The model doesn't include complex interactions of the tires with the road, i.e., it doesn't model dynamics of the vehicle.

![Kinematic model equations](media/kinematic_model.png)

The software implementation looks like the following:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where:

`x, y` - car's position coordinates,
`psi` - car's heading direction,
`v` - car's velocity,
`cte` - cross-track error,
`epsi` - error in heading direction

These are considered as inputs of the model, where with the help of `Ipopt` solver and `CppAD` given model's constrains like acceleration and steering angle limits, we can find our optimum control outputs:

`a` - car's acceleration (throttle),
`delta` - steering angle

The control outputs are optimal when the solution minimizes our cost functions which are defined as a sum of squared errors cross-track error, steering angle error, error in reference velocity, errors in use of actuators and the rate of their use.

### Timestep Length and Elapsed Duration (N & dt)

The number of points `N` and the time interval of their calculation `dt` define the model-based control prediction horizon = `N * dt` seconds. In my case the optimal prediction horizon was 1 second with the resolution on 0.1 second that resulted in 10 predicted waypoints computed. Increasing the horizon too much into the future leads to unnecessary computation since the world changes drastically on high speed. Increasing the step resolution on my machine was braking the controller not being able to meet the increased computational constraints. 100 milliseconds resolution is also an optimal value given the system respond delay of 100 ms. Computing faster than the system can react brings no benefit to the controller.

### Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are prepocessed by transorming them from the world's to vehicle's reference frame (see [lines 108-113](./src/main.cpp#L104)). Then they are fed to the [`polyfit`](./src/main.cpp#L116) function to find coefficients of a 3rd order polynomial.

### Model Predictive Control with Latency

The vehicle's actuation has a 100 ms latency. This latency was accounted for when computing the state vector based on the vehicle's kinematic model. This step can be found in [lines 122 through 131](./src/main.cpp#L120) with the resulting state vector found in [line 135](./src/main.cpp#L133).

## Simulation

### The vehicle must successfully drive a lap around the track.

The vehicle drives smoothly with the reference velocity of 100 MPH. The simulator run video can be found in [media/simulator_MPC.mov](./media/simulator_MPC.mov).

![Simulator run](media/simulator_MPC.gif)


