#ifndef Behavior_H
#define Behavior_H

#include <vector>
#include "Vehicle.h"
using namespace std;

// struct ModeData {
//   Vehicle car;
//   vector<Vehicle> cars;
//   double number;
//
//   ModeData(Vehicle car_in) : car(car_in), car_data(true), cars_data(false), num_data(false) {};
//   ModeData(vector<Vehicle> cars_in) : cars(cars_in), car_data(false), cars_data(true), num_data(false) {};
//   ModeData(double num) : number(num), car_data(false), cars_data(false), num_data(true) {};
//
//
//
// private:
//   bool car_data
//   bool cars_data
//   bool num_data
// }

struct ActiveMode {
  string mode;
  vector<double> state;
  double number;

  ActiveMode(string mode, vector<double> state) : mode(mode), state(state), number(0) {};
  ActiveMode(string mode, double number) : mode(mode), number(number), state(vector<double>(3)) {};
};


struct SearchMode {
  vector<ActiveMode> activeModes;
  int goal_lane;
  double min_lv_speed = -1;
};


#endif
