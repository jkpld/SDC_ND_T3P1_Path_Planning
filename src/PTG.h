#ifndef PTG_H
#define PTG_H

#include <vector>
#include <string>
#include <math.h>
#include <random>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Vehicle.h"
#include "Trajectory.h"
#include "TrajectoryCost.h"

class PTG {
public:

  vector<int> N_SAMPLES = {7, 1, 1}; // t, s, d
  vector<double> STEP = {0.5, 1.5, 0.75, 0, 0, 0, 0};
  // vector<double> SIGMA = {2, 10, 1, 4, 0.1, 2, 0.1} // t, s, d, s_d, d_d, s_dd, d_dd

  PTG() {};
  virtual ~PTG() {};

  /* JMT - generate a 1D jerk-minimizing-trajectory with initial conditions
    start, final conditions, end, and that takes time T to complete.
    start : (s0, ds0/dt, d2s0/dt2)
    end : (s1, ds1/dt, d2s1/dt2)
    T : time
  */
  vector<double> JMT(vector<double> start, vector<double> end, double T) {
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
         end[1]-(start[1]+start[2]*T),
         end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
      result.push_back(C.data()[i]);
    }

    return result;
  };

  /* generate - generate a set of possible trajectories
    our_car :
    target : a target car
    cars : a vector of all of the cars near our_car
    output : vector of Trajectory, the last Trajectory has the lowest cost.

  */
  Trajectory generate(Vehicle our_car, Vehicle target, double T, vector<Vehicle>& cars) {

    int rt = floor(N_SAMPLES[0]/2);
    int rs = floor(N_SAMPLES[1]/2);
    int rd = floor(N_SAMPLES[2]/2);

    auto state0 = our_car.traj.evaluate_state(0.0); // cartesian

    // cout << "a" << endl;

    // Start states
    vector<double> s0 = {state0[1],state0[3],state0[5]};
    vector<double> d0 = {state0[2],state0[4],state0[6]};

    // cout << "b" << endl;

    vector<Trajectory> trajectories;
    double minCost = 999999;
    int counter = 0;
    int best_traj_idx;
    Trajectory best_traj;

    Vehicle testTarget = target;

    // iterate over steps in time
    for (int tt = -rt; tt <= rt; tt++) {
      // for each step in time compute all possible combinations of s and d
      // perturbations

      double T_t = T + tt*STEP[0];
      auto goal_t = target.traj.evaluate_state(T_t); // Frenet

      for (int ss = -rs; ss <= rs; ss++) {
        for (int dd = -rd; dd <= rd; dd++) {

          // perturb goal
          vector<double> goal_ti (goal_t);
          goal_ti[1] += ss*STEP[1];
          goal_ti[2] += dd*STEP[2];

          // get goal state in cartesian coordinates
          testTarget.set_state(goal_ti);
          goal_ti = testTarget.state;
          // cout << "c, ";


          // goal states
          vector<double> s1 = {goal_ti[1],goal_ti[3],goal_ti[5]};
          vector<double> d1 = {goal_ti[2],goal_ti[4],goal_ti[6]};

          // trajectory computation
          auto s_traj = JMT(s0, s1, T_t);
          auto d_traj = JMT(d0, d1, T_t);
          Trajectory traj = Trajectory(s_traj, d_traj, T_t);

          cout << "goal :";
          for (auto i : goal_ti) cout << " " << i;
          cout << endl;
          cout << "x_traj :";
          for (auto i : s_traj) cout << " " << i;
          cout << endl;
          cout << "y_traj :";
          for (auto i : d_traj) cout << " " << i;
          cout << endl;
          cout << "T_t : " << T_t << endl;

          // auto trajT = traj.evaluate_state(0);
          // cout << "trajectory start sd : " << counter;
          // for (auto i : trajT) cout << " " << i;
          // cout << endl;
          //
          // trajT = traj.evaluate_state(traj.T);
          // cout << "trajectory end sd : " << counter;
          // for (auto i : trajT) cout << " " << i;
          // cout << endl;


          our_car.traj = traj; // assign trajectory to temporary car for computing cost

          // cout << "d, ";

          // compute trajectory cost
          TrajectoryCost trajCost = TrajectoryCost(our_car, target, T, cars);
          traj.cost = trajCost.compute();
          traj.valid = trajCost.is_valid();

          // cout << "Cost : ";
          // cout << traj.cost << endl << endl;

          // cout << "e, ";

          // compare cost and add to stack
          if ((minCost == 999999) || (traj.cost < minCost)) {
            minCost = traj.cost;
            best_traj_idx = counter;
            best_traj = traj;
          }

          // cout << counter << endl;
          ++counter;
          trajectories.push_back(traj);
        }
      }
    }

    // Move the best trajectory to the end of the trajectories vector
    trajectories.erase(trajectories.begin() + best_traj_idx); // remove it
    trajectories.push_back(best_traj); // add to back

    return best_traj;
    // return trajectories;
  };

  // Create a set of goals around the given location
  // vector<vector<double>> perturb_goal(vector<double> goal);
};

/*
vector<vector<double>> PMT::perturb_goal(vector<double> goal) {

  vector<vector<double>> goals;
  default_random_engine generator; // initialize random number generator

  // iterate over each parameter and create N_SAMPLES additional goals
  for (int k; k<7; k++) {

    normal_distribution<double> s_gen(goal[k], SIGMA[k]);

    vector<double> goal_k; // create vector of goals for parameter_k
    goal_k.push_back(goal[k]); // add the original

    for (int i=0; i<N_SAMPLES; i++) {
      goal_k.push_pack(s_gen(generator));
    }

    goals.push_back(goal_k); // add the parameters to the stack
  }
  return goals;
};
*/

#endif
