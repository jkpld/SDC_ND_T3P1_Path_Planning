#include "Vehicle.h"

const double NEIGHBORHOOD_RANGE2 = 10*10;
const double TIME_HORIZON = 1;

Vehicle::Vehicle() {
  ID = 999;
  state.assign(7,0);
}
Vehicle::Vehicle(int id0, vector<double> sm0) {
  ID = id0;
  state.assign (7,0);

  // update current state position and velocity
  for (int i=0; i<5; i++) {
    state[i] = sm0[i];
  }
}
Vehicle::~Vehicle() {}

void Vehicle::pushState(vector<double> sm1) {
  double dt = (sm1[0] - state[0])/1000.0;

  if (dt > 1) {
    // If the previous state is very old, then reinitialize the state
    for (int i=0; i<5; i++) {
      state[i] = sm1[i];
    }
    state[5] = 0;
    state[6] = 0;

  } else {
    // update state acceleration
    state[5] = (sm1[3] - state[3]) / dt; // ax
    state[6] = (sm1[4] - state[4]) / dt; // ay

    // update current state position and velocity
    for (int i=0; i<5; i++) {
      state[i] = sm1[i];
    }
  }
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

void Vehicle::update_neighborhood(map<int,Vehicle>& cars) {
  /*
  Go through each vehicle and determine if it is within NEIGHBORHOOD_RANGE at
  either its current location or its location at TIME_HORIZON. If it is in the
  range, then add its ID to the list of cars in the neighborhood
  */

  // clear the current neighborhood
  neighborhood.clear();

  vector<vector<double>> myStates;
  for (double dt = 0; dt <= TIME_HORIZON; dt += TIME_HORIZON/5.0) {
    myStates.push_back(this->predict(dt));
  }

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
};
