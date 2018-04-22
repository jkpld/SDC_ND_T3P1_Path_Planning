# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Overview
In this project, I implemented the method outlined in the paper "Optimal Trajectory Generation for Dynamic Street Scenarios in the Frenet Frame" by Moritz Werling, Julius Ziegler, Soren Kammel, and Sebastian Thrun, (2010). This method will be discussed below.

I first, created a simulator in Matlab to develop the code. The simulator allows for reproducible scene construction, which greatly eases testing. Using matlab I created the trajectory generation and behavioral planning modules, along with a reactive layer module. The matlab code was then ported over to c++.

More detailed methods are described below and a description of the code is provided at the bottom.

### Result
The car successfully drives without incident.

![Alt Text](/images/IMG_3406.JPG)

## Optimal trajectory generation
All details of the method are given in the paper mentioned above. In short, a set of jerk-minimizing-trajectories (JMT) in the lateral direction are combined with a set of JMT is the longitudinal direction and the combination with the lowest cost is used as the final 2d trajectory. (This is implemented in `TrajectoryGenerator/generate_()`.)

The longitudinal trajectories are created using several different possible modes, *following*, *merging*, *stopping*, and *velocity keeping*. The first three modes solve for a full 5th order jerk-minimizing polynomial. The *velocity keeping* method, only cares about maintaining a speed and not any position, therefore, the speed trajectory can be solved for (a 4th order polynomial) and then integrated to get the positional trajectory. (The code for these methods are contained in `TrajectoryGenerator/ JMT(), JMT_vel_keep(), following(), merging(), stopping(), velocity_keeping()`.)

The lateral trajectories can be divided into two groups for slow and fast cars. For this project I only implement the code for fast cars. (`TrajectoryGenerator/lateral()`)

The validity of the individual lateral and longitudinal trajectories are determined by looking to see if their maximum speed/accerlation/jerk are above the maximum values. (`TrajectoryGenerator/compute_1d_validity()`) This completed by evaluating the trajectory derivatives at the roots of the derivatives. (The code contained in `polynomial.h`, see below for description.)

All combinations of the remaining lateral and longitudinal trajectories are then combined and sorted from lowest cost to highest cost. I then iterate through each combination and use the first combination that is valid. Validity is determined (`TrajectoryGenerator/compute_2d_validity()`) by ensuring the speed in Cartesian coordinates is below the maximum speed and by ensuring our car does not collide with any other car. Car collision is iteratively performed. First the distance between the car centers is used to rule out the possibility of collisions in most cases. When it does not rule out a collision, first the axis-aligned bounding boxes are used to determine if there is a collision; if a collision is still not ruled out, then the separating axis theorem is used to full determine if the two cars collide or not. (The axis-aligned and object-aligned bounding box collision is implemented in the `helpers/Rectangle` class.). Finally, I accept the first trajectory combination that is valid.

In the main() function, I then cyclically re-plan the trajectories every 0.2 seconds.

## Behavioral module
The Behavioral planning (`BehavioralModule.h`) is very simple. If there is no car in front of us, then I use `velocity_keeping`. If there is a car in front, then I switch to `following`. If the car in front is slower than us, then I search the other lanes, using `velocity_keeping`, `following`, and `merging`. The desion of which lane to go to uses a very simple cost function based on the final speed of the trajectory, the slowest car in front of us in other lanes, and a cost for changing lanes. (`BehaviorModule/compute_cost()`).

## Reactive layer
If the behavioral module for fails to generate a trajectory that does not end up in a collision, then a reactive layer is called that searchers across all lanes with a large search grid of end speeds.

## Surrounding cars
The car trajectories are simply estimated using the last two measurements, which allows for the acceleration to be estimated. I then assume that each car follows a constant acceleration trajectory.

## Code
The code contains several header only files
* `TrajectoryGenerator` : Contains the functions and the constants for performing jerk-minimizing-trajectory generation.
* `Vehicle.h` : Class `Vehicle` with data properties for the car's state, and trajectory as well as methods for obtaining the car's state, trajectory, location, and bounding box as a particular time.
* `BehaviorModule` : Computes possible trajectories based on the surrounding cars and chooses the lane that minimizes the cost.
* `RoadMap.h` : Class `RoadMap` that stores the road's waypoints as well as a spline interpolant of the waypoints (wrapped in a piece-wise polynomial `PP`). The class contains methods for obtaining the tangential and normal unit vectors (returned as transformation matrices), and the road's curvature.
* `polynomial.h` : A set of functions for working with polynomials. Namely, computing derivatives and integrals, polynomial multiplication, polynomial root finding (by the eigenvalues of the companion matrix), evaluating all derivatives of the polynomial at a given point, and a piece-wise polynomial class `PP`.
* `helpers.h` : A set of functions and classes to help. `Trajectory` class for storing trajectories, `State` class for storing car states, `Rectangle` class for string rectangles an determining if they overlap or not, ...
* `pchip.h` : A class for computing shape-preserving cubic hermite polynomials. This was used for road interpolation before splines were used (for continuous curvature).

## Matlab simulator
The matlab development code has the following structure.
* The `CarSim` class contains the simulation. The simulation is created by passing in a set of cars with predefined states. This makes it very easy for debugging as we don't have to wait for some situation to randomly occur, as with the c++ simulation.
* The class `PathPlanner` is equivalent to the c++ main.cpp file and the method `PathPlanner.generatePath` is what runs (equivalent to the h.onMessage) and takes in the data, ego_pose, sensor_fusion (cars), and the previous_path. In addition it can take in another set of paths, that is very helpful when debugging.
* The script `run_car_simulation` is what runs the `CarSim` and the `PathPlanner` asynchronously (thus, this requires the parallel computing toolbox)
* Note, the simulation just uses frenet coordinates (the road is a straight line), so we only need to worry about the trajectory generation and behavior planning.

Here is an example of the simulator and the behavior planner
![Alt Text](/images/test_merging.gif)
![Alt Text](/images/test_merging2.gif)
