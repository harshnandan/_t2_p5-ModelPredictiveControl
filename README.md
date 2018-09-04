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
  * Run either 'install-mac.sh' or 'install-ubuntu.sh'.
  * If you install from source, checkout to commit 'e94b6e1', i.e.
    '''
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    '''
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: 'mkdir build && cd build'
3. Compile: 'cmake .. && make'
4. Run it: './mpc'.

## Project Instructions and Rubric

### Compilation

The code compiles without any errors or warnings.

### Implementation

A kinematic model of motion is used and it neglects any kinetic behavior like interaction between road-tire or mass etc. The equations are:

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi[t+1] = psi[t] - v[t] / Lf * delta[t] * dt

v[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt


Where:

- 'x, y' : Car's position
- 'psi' : Car's heading 
- 'v' : Car's velocity
- 'cte' : Cross-track error
- 'epsi' : Orientation error

Those values are considered the state of the model. In addition to that, 'Lf' is the distance between the car of mass and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

- 'a' : Car's acceleration (throttle)
- 'delta' : Steering angle, considering sign convention from the simulator, signs in equations with 'delta' have been flipped

The objective of this problem is to find 'a' and 'delta' such that an objective function designed to make the car follow the is minimized. The objective function contains following components:

- Square sum of 'cte', 'epsi' and 'velocity error'. It could be found [here](./src/MPC.cpp#L54).
- Square sum of the actuator action to penalize excessive use of actuation. It could be found [here](./src/MPC.cpp#L61).
- Square sum of the rate of actuator action to penalize sudden change in actuation. It could be found [here](./src/MPC.cpp#L68).

A higher factor has been used on rate of actuation component as it was initially observed that the car was careening a lot while following the desired trajectory. 

### Timestep Length and Elapsed Duration (N & dt)

The two hyper-parameters define the prediction horizon. Choosing a 50 milisecond dt and N of 10 provided good balance between computational demand and control performance.

### Polynomial Fitting and MPC Preprocessing

The waypoints supplied by the simulator are in map coordinate system, these were first converted to car coordinate system and then a 3rd order polynomial is fitted to waypoints in vehicle coordinate system. 

### Model Predictive Control with Latency
To handle actuator latency the state at the end of delay is used as opposed to initial state. 

## Simulation

The successfully drive laps around the track.



