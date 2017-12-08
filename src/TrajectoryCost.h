#ifndef TrajectoryCost_h
#define TrajectoryCost_h

#include "Trajectory.h"
#include "Vehicle.h"
#include <vector>
#include <math.h>

class TrajectoryCost {
public:
  Vehicle our_car;
  Trajectory traj;
  Vehicle target;
  vector<Vehicle> cars;
  double T;

  // Weights for the different cost functions
  vector<double> const STATE_DIFF_WEIGHT = {0.7, 1, 1, 0.5, 0.2, 0, 0}; // {t, s, d, vs, vd, as, ad}
  // vector<double> const STATE_DIFF_WEIGHT = {0.7, 1, 1, 0.5, 0.5, 0, 0}; // {t, x, y, vx, vy, ax, ay}
  double const BUFFER_WEIGHT = 0.7;
  double const EFFICIENCY_WEIGHT = 1;
  double const AVG_ACCEL_WEIGHT = 0.7;
  double const AVG_JERK_WEIGHT = 0.7;

  // Normalization constants used to scale the logistic function
  double const VEHICLE_SIZE_NORM = 5; // [m]
  vector<double> const STATE_DIFF_NORM = {0.2, 2, 1, 5, 5, 5, 5}; // {t, s, d, vs, vd, as, ad}, [s, m, m, m/s, m/s, m/s/s, m/s/s]
  // vector<double> const STATE_DIFF_NORM = {0.2, 2, 2, 5, 5, 5, 5}; // {t, x, y, vx, vy, ax, ay}, [s, m, m, m/s, m/s, m/s/s, m/s/s]
  double const AVG_ACCEL_NORM = 1;
  double const AVG_JERK_NORM = 2;

  // Constants determining trajectory validity
  double const MIN_DIST = 0.2;
  double const MAX_ACCEL = 10;
  double const MAX_JERK = 10;

  TrajectoryCost(Vehicle r_cr, Vehicle trgt, double t, vector<Vehicle>& crs);

  // compute the cost
  double compute();
  // determine if trajectory is valid (collision, max accel, max jerk)
  bool is_valid();

  /* Helpers */
  double logistic(double x) {return 2.0/(1.0 + exp(-x)) - 1.0;};
  double nearest_approach(Vehicle car, bool detect_collision);

  /* Cost functions */
  double state_diff_cost();
  double buffer_cost();
  double efficiency_cost();
  double avg_accel_cost();
  double avg_jerk_cost();

  bool collision_cost();
  bool max_accel_cost();
  bool max_jerk_cost();
private:

  MatrixXd traj_xy;
  MatrixXd targ_xy;
  MatrixXd accel_xy;
  MatrixXd jerk_xy;
};

/* ------------------------------------------------------------------------- */
/* Primary functions */
/* ------------------------------------------------------------------------- */

inline TrajectoryCost::TrajectoryCost(Vehicle r_cr, Vehicle trgt, double t, vector<Vehicle>& crs)
  : our_car(r_cr), target(trgt), cars(crs), T(t) {

  // initialize some variables that are needed in more than one cost function
  // or several times
  // cout << "differentiating ... ";
  auto acc = our_car.traj.differentiate(2);
  auto jerk = acc.differentiate();
  // cout << " finished" << endl;

  // cout << "acc x_traj :";
  // for (auto i : acc.x_coeffs) cout << " " << i;
  // cout << endl;
  // cout << "acc y_traj :";
  // for (auto i : acc.y_coeffs) cout << " " << i;
  // cout << endl;

  // cout << "jerk x_traj :";
  // for (auto i : jerk.x_coeffs) cout << " " << i;
  // cout << endl;
  // cout << "jerk y_traj :";
  // for (auto i : jerk.y_coeffs) cout << " " << i;
  // cout << endl;

  // cout << "computing accel ...";
  accel_xy = acc.generate_EigMat(acc.T/100);
  // cout << " finished" << endl;
  // cout << "computing jerk ...";
  jerk_xy = jerk.generate_EigMat(jerk.T/100);
  // cout << " finished" << endl;

  // cout << "computing trajectories ...";
  traj_xy = our_car.traj.generate_EigMat(our_car.traj.T/100);
  targ_xy = target.traj.generate_EigMat(our_car.traj.T/100);
  // cout << " finished" << endl;
};

inline bool TrajectoryCost::is_valid() {
  // cout << "Computing validity ...";
  bool output = collision_cost() && max_accel_cost() && max_jerk_cost();
  // cout << " finished" << endl;
  return output;
};

inline double TrajectoryCost::compute() {
  // cout << "Computing cost ...";
  // compute cost
  double cost = 0;
  cost += state_diff_cost();
  cost += buffer_cost();
  cost += efficiency_cost();
  cost += avg_accel_cost();
  cost += avg_jerk_cost();
  // cout << " finished" << endl;
  return cost;
};

/* ------------------------------------------------------------------------- */
/* Helpers */
/* ------------------------------------------------------------------------- */

inline double TrajectoryCost::nearest_approach(Vehicle car, bool detect_collision = true) {
  // if detect_collision is false, then the distance computed is relative to the
  // centers of the cars.
  // if detect_collision is true, then the distance is computed between all
  // possible combinations of one corner of one car and one corner of the other
  // car.
  auto diff = traj_xy - targ_xy;
  double dist = 0;
  if (detect_collision) {
    // compute the nearest distance between any 2 corners of the 2 cars
    double cw1 = our_car.car_size[0];
    double cl1 = our_car.car_size[1];
    double cw2 = car.car_size[0];
    double cl2 = car.car_size[1];

    auto min_dist = 999999;
    // iterate over each corner of both cars
    for (int w1 = -cw1/2; w1 < cw1; w1 += cw1) {
      for (int l1 = -cl1/2; l1 < cl1; l1 += cl1) {
        for (int w2 = -cw2/2; w2 < cw2; w2 += cw2) {
          for (int l2 = -cl2/2; l2 < cl2; l2 += cl2) {

            Vector2d offset;
            offset << w1-w2, l1-l2;

            dist = (diff.rowwise() - offset.transpose()).rowwise().squaredNorm().minCoeff();
            if (dist < min_dist) min_dist = dist;

          }
        }
      }
    }
  } else {
    dist = diff.rowwise().squaredNorm().minCoeff();
  }
  return sqrt(dist);
};

/* ------------------------------------------------------------------------- */
/* Cost functions */
/* ------------------------------------------------------------------------- */
inline double TrajectoryCost::state_diff_cost() {
  // target and trajectory at the trajectory's end time
  vector<double> targ_t = target.traj.evaluate_state(our_car.traj.T);
  vector<double> traj_t = our_car.traj.evaluate_state(our_car.traj.T);

  // cost for time difference
  double cost = STATE_DIFF_WEIGHT[0] * logistic(fabs(our_car.traj.T - T) / STATE_DIFF_NORM[0] );

  // cost for all other state variables.
  for (int i=1; i<7; ++i) {
    cost += STATE_DIFF_WEIGHT[i] * logistic(fabs(targ_t[i] - traj_t[i]) / STATE_DIFF_NORM[i] );
  }

  return cost;
};

inline bool TrajectoryCost::collision_cost() {
  double min_dist = 999999;
  for (Vehicle car : cars) {
    double dist = nearest_approach(car);
    if (dist < min_dist) min_dist = dist;
  }

  return (min_dist > MIN_DIST);
};

inline double TrajectoryCost::buffer_cost() {
  double min_dist = 999999;
  for (Vehicle car : cars) {
    double dist = nearest_approach(car, false);
    if (dist < min_dist) min_dist = dist;
  }

  return BUFFER_WEIGHT * logistic(VEHICLE_SIZE_NORM / (min_dist + 0.00001));
}

inline double TrajectoryCost::efficiency_cost() {
  // use the fractional differance between target s_speed and trajectory s_speed
  vector<double> targ_t = target.traj.evaluate_state(our_car.traj.T);
  vector<double> traj_t = our_car.traj.evaluate_state(our_car.traj.T);
  Vector2d v_targ, v_traj;
  v_targ << targ_t[3], targ_t[4];
  v_traj << traj_t[3], traj_t[4];

  return EFFICIENCY_WEIGHT * logistic( (v_targ - v_traj).norm() / v_targ.norm() );
}

inline double TrajectoryCost::avg_accel_cost() {
  double avg_accel = accel_xy.rowwise().norm().mean();
  return AVG_ACCEL_WEIGHT * logistic(avg_accel / AVG_ACCEL_NORM);
}

inline double TrajectoryCost::avg_jerk_cost() {
  double avg_jerk = jerk_xy.rowwise().norm().mean();
  return AVG_JERK_WEIGHT * logistic(avg_jerk / AVG_JERK_NORM);
}

inline bool TrajectoryCost::max_accel_cost() {
  double max_accel = accel_xy.rowwise().norm().maxCoeff();
  return (max_accel < MAX_ACCEL);
}

inline bool TrajectoryCost::max_jerk_cost() {
  double max_jerk = jerk_xy.rowwise().norm().maxCoeff();
  return (max_jerk < MAX_JERK);
}

#endif
