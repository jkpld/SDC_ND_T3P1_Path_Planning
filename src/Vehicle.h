#ifndef Vehicle_H
#define Vehicle_H

#include <vector>
#include <map>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Trajectory.h"
#include "RoadMap.h"

using namespace std;
using namespace Eigen;

class Vehicle {
public:


  /* Vehicle state_measured, s
    s[0] : time
    s[1] : x
    s[2] : y
    s[3] : vx
    s[4] : vy
  */
  /* Vehicle state, s
    s[0] : time
    s[1] : x
    s[2] : y
    s[3] : vx
    s[4] : vy
    s[5] : ax
    s[6] : ay
  */
  vector<double> state; // state in frenet coordinates
  vector<double> state_xy; // state in cartesian coordinates

  RoadMap road;

  // Predicted trajectory
  Trajectory traj;
  Trajectory traj_xy;

  // Car ID
  int ID;

  // Car size - width, length
  vector<double> car_size = {2, 4.8};

  // Cars in neighborhood
  vector<int> neighborhood;

  // Constructor, Destructor
  Vehicle();
  Vehicle(RoadMap& road);
  Vehicle(RoadMap& road, int id0);
  Vehicle(RoadMap& road, int id0, vector<double> sm0);
  virtual ~Vehicle();

  // Add new measurment of state
  void pushState(vector<double> sm1);

  // Predict trajectory
  void predict_trajectory();

  // Predict where the car will be in T sec from current state
  vector<double> predict(double T);
  vector<double> predict(double T, vector<double> state);
  vector<vector<double>> generate_predicted_trajectory();
  vector<vector<double>> generate_predicted_trajectory(vector<double> state);

  // Update list of cars in neighborhood
  void update_neighborhood(map<int, Vehicle>&);

  void set_state(vector<double>&);
  void set_state_xy(vector<double>&);
private:
  void assign_traj();
};


#endif /* Vehicle */
