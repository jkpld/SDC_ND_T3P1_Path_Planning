#ifndef Vehicle_H
#define Vehicle_H

#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"

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

  vector<double> state;

  // Car ID
  int ID;

  // Cars in neighborhood
  vector<int> neighborhood;

  // Constructor, Destructor
  Vehicle();
  Vehicle(int id0, vector<double> sm0);
  virtual ~Vehicle();

  // Add new measurment of state
  void pushState(vector<double> sm1);

  // Predict where the car will be in T sec from current state
  vector<double> predict(double T);

  // Update list of cars in neighborhood
  void update_neighborhood(map<int, Vehicle>&);

};


#endif /* Vehicle */
