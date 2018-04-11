# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

![screenshot][0]
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


## Make Sense of the Code Structure
### Variables
1. We have a state vector of size 6, [x, y, psi, v, cte, epsi]
2. We have a acuator vector of size 2, [delta, acceleration]
3. If we have N time steps and each time step is dt second long, the entire time span during which we predict the vehicle trajectory is N * dt second long.
4. `vector<double> vars` is a vector that stores state and actuator variables. Its size is `state.size() * N + (N - 1) * actuator.size()`. You can think of the values store in the vector as the initial values. This vector is treated as a set of independent variables by **IPOPT**.
5. `vector<double> vars_lowerbound`, `vector<double> vars_upperbound`, `vector<double> constraints_lowerbound` and `vector<double> constraints_upperbound`. The first 2 vectors are constraints on individual variables. The rest 2 are constraints on equation values that involve variables in `vars`. Often in optimization problems you have one objective function you try to minimize and a set of constraint equations or inequalities
 that involve variables in the objective function you cannot violate. `constraints_lowerbound` and `constraints_upperbound` store the range of values that those equations and inequalities can be.
 6. `vector<double> fg` is a vector that contains the objective function and the contraints by putting the results of arithmetic combinations of `vars` elements. All elements from `vars` are treated as variables and others are treated as numbers. 

### Procedures
1. Polynomial fitting. The simulator provides you with the waypoints that you use to fit polynomial curve to. This is your reference trajectory
2. Mind the error terms. Refer to lectures to see how `CTE` and `epsi` are computed. It is implicitly assumed that dt is sufficiently small such that vehicle is moving in straight line during dt.

### Pitfalls
1. Incorporate latency. To incorporate latency you need to compute the state of the vehicle after the latency. Use that as the initial state fed to `MPC::Solve`. Note we repeat the MPC computation at each time step. So you only need to consider latency for the current time step.
2. Transform coordinates between car coordinate system and map coordinate system. In this project transforming between coordinate systems is no different than [the kidnapped vehicle](https://github.com/Xiaohong-Deng/CarND-Kidnapped-Vehicle-Project). This time we need to transform data from the map system to the vehicle system. psi is the vehicle heading relative to the map coordinate system. x, y is the vehicle coordinates in the map system. You can pass the data either in map system coordinates or in vehicle system coordinates, but to visualize you need the coordinates in
the vehicle system. Transform the coordinates before feeding them to `MPC::Solve`
3. Tweaking time steps `N` and duration `dt`. Too large N will make computation expensive. Too small dt will essentially increase N given the horizon N*dt is fixed. If dt is set to a larger value during dt the surroundings of the vehicle may have changed such that the trajectory is no longer feasible.
4. psi(t+1) - psi(t) < 0 is considered turning right. But the simulator considers that a positive value indicates turning right. For psi(t+1) - psi(t) = A*delta(t), we feed delta(t) to the simulator. So we need to flip the sign of delta(t) to preserve the correct steering angle.
5. When incorporating latency we need to update new psi using steering angle fed by the simulator. That also requires sign flipping. Namely `steer_angle = -steer_angle` otherwise it will lead swinging waypoints fitted polynomial.

### Choice of incorporating latency
There are two ways of incorporating latency. In my implementation I chose the first one.
1. In this case we think of the vehicle state after latency as the origin of the vehicle system. First calculate coordinates of the vehicle in the map system and its psi after latency together with speed. Based on that updated state transform the waypoints to the vehicle system. Then feed state to solver where (x, y, psi) = (0, 0, 0).
2. In this case we think of the vehicle state before latency as the origin of the vehicle system. First we transform waypoints to the vehicle system. Then we compute the vehicle state after latency. When we feed state to the solver we have nonzero x, y and psi.

[0]: ./sh-2018-04-11.png
