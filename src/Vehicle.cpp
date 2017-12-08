#include "Vehicle.h"

const double NEIGHBORHOOD_RANGE2 = 20*20;
const double TIME_HORIZON = 1;

Vehicle::Vehicle() {
  road = RoadMap();
  ID = 999;
  state.assign(7,0);
  state_xy.assign(7,0);
};
Vehicle::Vehicle(RoadMap& road_in) : road(road_in), ID(999) {
  state.assign(7,0);
  state_xy.assign(7,0);
  state[0] = -2000;
  state_xy[0] = -2000;
}
Vehicle::Vehicle(RoadMap& road_in, int id) : road(road_in), ID(id) {
  state.assign(7,0);
  state_xy.assign(7,0);
  state[0] = -2000;
  state_xy[0] = -2000;
}
Vehicle::Vehicle(RoadMap& road_in, int id0, vector<double> sm0)  : road(road_in), ID(id0) {
  state.assign(7,0);
  state_xy.assign(7,0);

  // update current state position and velocity
  for (int i=0; i<5; i++) {
    state_xy[i] = sm0[i];
  }

  state = road.Cartesian_to_Frenet(state_xy);
  predict_trajectory();
}
Vehicle::~Vehicle() {}

void Vehicle::predict_trajectory() {
  /* Eventually can put something in that predicts if the car is planning to
    stay in the same lane or change lanes based on its state.
    For now, simply predict that the car will move in a parabola.
  */
  assign_traj();
}

void Vehicle::assign_traj() {
  traj_xy.x_coeffs = {state_xy[1], state_xy[3], state_xy[5]};
  traj_xy.y_coeffs = {state_xy[2], state_xy[4], state_xy[6]};
  traj_xy.T = TIME_HORIZON;

  traj.x_coeffs = {state[1], state[3], state[5]};
  traj.y_coeffs = {state[2], state[4], state[6]};
  traj.T = TIME_HORIZON;
};

void Vehicle::pushState(vector<double> sm1) {
  double dt = (sm1[0] - state_xy[0])/1000.0;

  if (dt > 0.2) {
    // If the previous state_xy is very old, then reinitialize the state_xy
    for (int i=0; i<5; i++) {
      state_xy[i] = sm1[i];
    }
    state_xy[5] = 0;
    state_xy[6] = 0;

  } else {
    // update state_xy acceleration
    state_xy[5] = (sm1[3] - state_xy[3]) / dt; // ax
    state_xy[6] = (sm1[4] - state_xy[4]) / dt; // ay

    // update current state_xy position and velocity
    for (int i=0; i<5; i++) {
      state_xy[i] = sm1[i];
    }
  }

  state = road.Cartesian_to_Frenet(state_xy);
  predict_trajectory();
}

vector<double> Vehicle::predict(double T) {
  vector<double> s (state);
  s[0] = state[0] + T;
  s[1] = state[1] + state[3]*T + state[5]*T*T/2;
  s[2] = state[2] + state[4]*T + state[6]*T*T/2;
  s[3] = state[3] + state[5]*T;
  s[4] = state[4] + state[6]*T;
  return s;
}

vector<double> Vehicle::predict(double T, vector<double> state) {
  vector<double> s (state);
  s[0] = state[0] + T;
  s[1] = state[1] + state[3]*T + state[5]*T*T/2;
  s[2] = state[2] + state[4]*T + state[6]*T*T/2;
  s[3] = state[3] + state[5]*T;
  s[4] = state[4] + state[6]*T;
  return s;
}

vector<vector<double>> Vehicle::generate_predicted_trajectory() {
  vector<vector<double>> states;
  for (double dt = 0; dt <= TIME_HORIZON; dt += TIME_HORIZON/5.0) {
    states.push_back(this->predict(dt));
  }
  return states;
}
vector<vector<double>> Vehicle::generate_predicted_trajectory(vector<double> state_in) {
  vector<vector<double>> states;
  for (double dt = 0; dt <= TIME_HORIZON; dt += TIME_HORIZON/5.0) {
    states.push_back(this->predict(dt, state_in));
  }
  return states;
}

void Vehicle::update_neighborhood(map<int,Vehicle>& cars) {
  /*
  Go through each vehicle and determine if it is within NEIGHBORHOOD_RANGE at
  either its current location or its location at TIME_HORIZON. If it is in the
  range, then add its ID to the list of cars in the neighborhood
  */

  // clear the current neighborhood
  neighborhood.clear();

  for (auto& v : cars) {

    // Predict where the car will be at times in the future
    for (double dt = 0; dt <= TIME_HORIZON; dt += TIME_HORIZON/5.0) {

      auto s = v.second.predict(dt);
      auto ms = this->predict(dt);
      auto dx = ms[1] - s[1];
      auto dy = ms[2] - s[2];

      if ((dx*dx + dy*dy) < NEIGHBORHOOD_RANGE2) {
        neighborhood.push_back(v.first);
        break;
      }
    }
  }
}

void Vehicle::set_state(vector<double>& s) {
  state = s;
  state_xy = road.Frenet_to_Cartesian(state);
  predict_trajectory();
}

void Vehicle::set_state_xy(vector<double>& s) {
  state_xy = s;
  state = road.Cartesian_to_Frenet(state);
  predict_trajectory();
}
