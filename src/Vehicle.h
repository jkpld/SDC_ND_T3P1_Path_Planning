#ifndef Vehicle_H
#define Vehicle_H

#include <vector>
// #include <map>
#include <iostream>
// #include <algorithm>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
// #include "Trajectory.h"
// #include "RoadMap.h"

#include "helpers.h"


using namespace std;
using namespace Eigen;

class Vehicle {
private:
  int ID;
  double Length;
  double Width;

public:
  State state;
  vector<Trajectory> trajectory;
  double t0; // Time at which the state and trajectory are defined relative t0.

  // Constructor, Destructor
  Vehicle() : Vehicle(0, 4.8, 1.8) {};
  Vehicle(int id) : Vehicle(id, 4.8, 1.8) {};
  Vehicle(double L, double W) : Vehicle(0, L, W) {};
  Vehicle(int id, double L, double W) : ID(id), Length(L), Width(W), t0(0), state(State()), trajectory(state.get_trajectory()) {};

  void display() {
    cout << "Vehicle (ID : " << ID << ")" << endl;
    cout << " Length = " << Length << endl;
    cout << " Width = " << Width << endl;
    cout << " State, " << endl;
    state.display();
    cout << " Trajectory x(t), ";
    trajectory[0].display();
    cout << " Trajectory y(t), ";
    trajectory[1].display();
  }

  // Set the vehicle state and update the coresponding trajectory
  void set_state(State s0) {
    state = s0;
    trajectory = state.get_trajectory();
  }

  // Set the vehicle trajectory and update the corresponding state
  void set_trajectory(vector<Trajectory> traj) {
    trajectory = traj;
    state = State(traj[0].state_at(0), traj[1].state_at(0));
  }

  // Set vehicle size
  void set_size(double length, double width) {
    Length = length;
    Width = width;
  }

  /* Add new measurment of state, should be kalman filter, but simply estimates
    acceleration given the current and the last measurement (if not too
    outdated)
    measured state contains {x,y,vx,vy} - in Frenet coordinates! */
  void new_measurment(vector<double> m, double t1) {
    double dt = (t1 - t0);
    Vector3d x, y;

    if (dt > 0.5) {
      // If the previous state_xy is very old, then reinitialize the state
      x << m[0], m[2], 0;
      y << m[1], m[3], 0;
    } else {
      // update state_xy acceleration
      x << m[0], m[2], (state.x(1)-m[2])/dt;
      y << m[1], m[3], (state.x(2)-m[3])/dt;
    }

    set_state(State(x,y)); // Set the state
    t0 = 0; // Update the time.
  }

  // Return the cars bounding box at time t.
  Rectangle bounding_box(double t, Vector2d safty_margin = Vector2d::Zero()) {
    auto loc = location(t);
    return Rectangle(Length+safty_margin(0),
                     Width+safty_margin(1),
                     orientation(t),
                     loc[0],
                     loc[1]);
  };

  template <typename T>
  Rectangle bounding_box(double t, T& loc, Vector2d safty_margin = Vector2d::Zero()) {
    return Rectangle(Length+safty_margin(0),
                     Width+safty_margin(1),
                     orientation(t),
                     loc[0],
                     loc[1]);
  };

  double orientation(double t) {
    /* Orientation of vehicle at time t.
    The end state of a trajectory is zero acceleration, so we can get the
    orientation at any time after the knot by looking at the orientation just
    before the knot. This helps with cases where the speed in one direction is
    zero. Note this should work for this project, but it should be changed out
    with the proper orientation equations, as given in the paper "Optimal
    Trajectory Generation for Dynamic Street Senarios in the Frenet Frame" */
    t = t - t0;
    double T = max(trajectory[0].knot, trajectory[1].knot);
    t = (t>=T-0.01) ? T-0.01 : t;

    Vector3d sx = trajectory[0].state_at(t);
    Vector3d sy = trajectory[1].state_at(t);
    return atan2(sy[1], sx[1]);
  };

  State state_at(double t) {
    return State(trajectory[0].state_at(t-t0), trajectory[1].state_at(t-t0));
  }

  vector<double> location(double t) {
    return {trajectory[0].ppval(t-t0), trajectory[1].ppval(t-t0)};
  }

  // Get the location at many times. The type could be VectorXd or a
  // vector<double>, for example.
  template <typename T>
  vector<vector<double>> location(T t) {
    vector<vector<double>> loc;
    for (int i = 0; i<t.size(); ++i) {
      loc.push_back(location(t(i)));
    }
    return loc;
  }

  template <typename T>
  MatrixXd location_Eig(T t) {
    MatrixXd loc(t.size(),2);
    for (int i = 0; i<t.size(); ++i) {
      auto loc_i = location(t(i));
      loc.row(i) << loc_i[0], loc_i[1];
    }
    return loc;
  }
};


#endif /* Vehicle */
