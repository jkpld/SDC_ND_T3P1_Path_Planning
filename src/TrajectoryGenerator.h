#ifndef TrajectoryGenerator_H
#define TrajectoryGenerator_H

/* TrajectoryGenerator
A trajectory generation class that is based on the paper "Optimal Trajectory
Generation for Dynamic Street Senarios in the Frenet Frame". (This will be
sited as [1].)
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "polynomial.h"
#include "roots.h"
#include "helpers.h"
#include "Vehicle.h"


using namespace Eigen;
using namespace std;

class TrajectoryGenerator {
public:
  /* Weights for cost function, as defined in the paper [1]*/
  double kj; // weight for the jerk cost
  double kt; // weight fot the time-to-complete cost
  double ks; // weight for the difference in s-coordinate cost
  double ksd; // weight for the difference in s-dot-coordinate cost
  double kd; // weight for the difference in d-coordinate cost

  double k_lat; // weight for lateral trajectories
  double k_lon; // weight for longitudinal trajectories

  /* Safty_margin - This will be used to ensure the cars do not collide
  or come to close to each other. */
  Vector2d Safty_Margin;

  /* Parameters for constant distance and constant time law. */
  double D0; // static distance required between cars at speed 0
  double tau; // time distance between cars

  /* Speed, acceleration, and jerk maximum values.
    These should be the highest possible values that the vehicle can function
    at; however, here, they will simply be the maximum values allowed by the
    project.
  */
  double Max_Speed;
  double Max_Accel;
  double Max_Jerk;

  /* Information on the road, that really shouldn't be here probably. */
  double Min_D;
  double Max_D;
  double Lane_Width;

  /* Time horizons */
  double Time_Horizon; // used in checking for collisions
  double Reactive_Layer_Time_Horizon;

private:
  /* Properties controling the search values for the new trajectories */
  ArrayXd T_; // times used in searching for new trajectories
  ArrayXd ds_; // offset longitudinal distances
  ArrayXd dsd_; // offset speeds
  ArrayXd d_; // offset lateral distances

public:
  // Constructor with default values
  TrajectoryGenerator() :
    kj(1),
    kt(10),
    ks(100),
    ksd(40),
    kd(2000),
    k_lat(1),
    k_lon(1),
    Safty_Margin((Vector2d()<<0.5,0.5).finished()),
    D0(1),
    tau(1),
    Max_Speed(22.35),
    Max_Accel(10),
    Max_Jerk(10),
    Min_D(0),
    Max_D(12),
    Lane_Width(1),
    Time_Horizon(3),
    Reactive_Layer_Time_Horizon(1),
    T_(ArrayXd::LinSpaced(3,1,3)),
    ds_(ArrayXd::LinSpaced(11,-5,5)),
    dsd_(ArrayXd::LinSpaced(5,-2,2)),
    d_(ArrayXd::LinSpaced(3,-1,1)) {};

  /* Generate the best trajectory for ego using the SearchMode while preventing
  collisions with the cars. If no trajectory is found, then false is returned;
  otherwise, true is returned.
  */
  bool generate(Vehicle& ego, vector<Vehicle> const& cars, SearchMode const& SM) const {
    return generate_(ego, cars, SM, d_, dsd_);
  }

private:
  bool generate_(Vehicle& ego, vector<Vehicle> const& cars, SearchMode const& SM, ArrayXd const& search_d, ArrayXd const& search_sd) const {

    vector<Trajectory> trajs_d, trajs_s;
    vector<double> costs_d, costs_s;
    State state0 = ego.state;

    // Search for the lateral trajectory
    lateral(state0, SM.goal_lane, search_d, trajs_d, costs_d);

    // If there were no valid lateral trajectories, then return false
    if (trajs_d.empty()) {
      return false;
    }

    // Search for the longitudinal trajectories
    for (int i=0; i<SM.activeModes.size(); ++i) {
      Mode mode = SM.activeModes[i].mode;

      if (mode == Mode::FOLLOWING ) {
        following(state0, SM.activeModes[i].state[0], trajs_s, costs_s);
      } else if (mode == Mode::MERGING ) {
        merging(state0, SM.activeModes[i].state[0], SM.activeModes[i].state[1], trajs_s, costs_s);
      } else if (mode == Mode::STOPPING ) {
        stopping(state0, SM.activeModes[i].number, trajs_s, costs_s);
      } else if (mode == Mode::VELOCITY_KEEPING ) {
        velocity_keeping(state0, SM.activeModes[i].number, search_sd, trajs_s, costs_s);
      }
    }

    if (trajs_s.empty()) {
      return false;
    }

    /* Get the linear index of the combination of lateral trajectories with
     longitudinal trajectories. Ex.

     The outer sum of the costs C[i,j] = costs_d[i] + costs_s[j].
     Sort this matrix and return the linear indices to the costs C[i,j] in
     ascending order. The linear indices are i*costs_d.size() + j.
    */
    auto idx = sort_outer_sum(costs_d, costs_s);
    int Nd = costs_d.size(); // number of d-trajectories
    int Ns = costs_s.size(); // number of s-trajectories

    // If there are no cars, then simply use the lowest cost trajectory
    // combination
    if (cars.empty()) {

      int di,si;
      ind2sub(Nd, idx[0], di, si);
      ego.set_trajectory({trajs_d[di], trajs_s[si]});
      return true;

    } else {
      // Check each trajectory combination to see if there are any collisions
      // with the cars, and use the first combination that does not have any
      // collisions.

      int Nt = ceil(Time_Horizon * 0.2) + 1;
      int Nc = cars.size();

      // Construct the car trajectories for collision detection and the minimum
      // distance between ego and each car at which there cannot be a collision
      VectorXd t = VectorXd::LinSpaced(Nt,0,Nt-1) / 0.2; // set the times to check for collisions
      MatrixXd car_traj(Nc*Nt, 2); // initialize matrix for storing car paths
      ArrayXd min_collision_free_dist(Nc);

      for (int ci=0; ci<Nc; ++ci) {
        // get the car locations
        car_traj.block(Nt,2,ci*Nt,0) = cars[ci].location_Eig(t);

        // half diagonal of the car
        min_collision_free_dist(ci) = (cars[ci].get_size()/2 + Safty_Margin).norm();
      }
      min_collision_free_dist += (ego.get_size()/2 + Safty_Margin).norm(); // add the half diagonal of ego
      min_collision_free_dist *= min_collision_free_dist; // square it so later we don't have to take square roots of the distances between the cars.

      bool collide = false;
      for (int ds_i=0; ds_i<Nd*Ns; ++ds_i) {
        // Get the indices of the current trajectory combination
        int di,si;
        ind2sub(Nd, idx[ds_i], di, si);

        // Set ego's trajectory
        ego.set_trajectory({trajs_d[di], trajs_s[si]});

        // Detect collisions
        collide = detect_collision(ego, cars, car_traj, t, min_collision_free_dist);
        if (!collide) break;
      }

      // If all trajectories collide, then set ego's state back to its starting
      // state and return false.
      if (collide) {
        ego.set_state(state0);
        return false;
      }
    }

    return true;
  }

  bool reactive(Vehicle& ego, vector<Vehicle> const& cars) const {
    return true;
  }

  bool detect_collision(Vehicle& ego, vector<Vehicle> const& cars, MatrixXd car_traj, VectorXd t, VectorXd min_collision_free_dist)const {

    // NOTE: I am using just axis aligned bounding box instead of the exact
    // bounding box. Switch to object aligned bounding box by chaning this flag
    // to true;
    bool OA_DETECTION = false;

    int Nc = cars.size();
    int Nt = t.size();

    MatrixXd ego_traj = ego.location_Eig(t);
    MatrixXd dist(Nt,Nc);
    for (int ci=0; ci<Nc; ++ci) {
      dist.col(ci) = (car_traj.block(Nt,2,ci*Nt,0) - ego_traj).rowwise().squaredNorm() / min_collision_free_dist(ci);
    }

    // Find all the points (car/time combinations) that we must check for a collision.
    Matrix<bool,Dynamic,Dynamic> C = dist.array() <= 1;

    if (C.any()) {
      // Get the indices of the points we must check
      VectorXi idx = VectorXi::LinSpaced(C.size(),0,C.size()-1);
      idx.conservativeResize(stable_partition(idx.data(), idx.data()+idx.size(), [&C](int i){return C(i);})-idx.data());

      // We have to check all of these points for a collision, however, we can
      // stop checking upon the first detected collision. Therefore, first sort
      // the indices in ascending order of the distance, and start going through
      // the points from the closest. This should help reduce work when there is
      // a collision.
      sort(idx.data(), idx.data()+idx.size(), [&dist](size_t i1, size_t i2) {return dist(i1) < dist(i2);});

      for (int ct_i=0; ct_i<idx.size(); ++ct_i) {
        int ti,ci;
        ind2sub(Nt,idx[ct_i],ti,ci);
        Rectangle rec1 = ego.bounding_box(t(ti), ego_traj.row(ti), Safty_Margin);
        Rectangle rec2 = ego.bounding_box(t(ti), car_traj.row(ci*Nt+ti), Safty_Margin);
        if (rec1.overlap(rec2, OA_DETECTION)) return true;
      }
    }
    return false;
  }

  void lateral(State const& state0, double const& goal_lane, ArrayXd const& search_d, vector<Trajectory>& trajs, vector<double>& costs) const {

    // Initial lateral state.
    Vector3d d0 = state0.y;

    // The goal state.
    double d_goal = (goal_lane - 0.5)*Lane_Width; // Goal d-coordinate.
    Vector3d dg;
    dg << d_goal, 0, 0;

    // The search d values.
    ArrayXd d = search_d + d_goal;

    for (int di =0 ; di<search_d.size(); ++di) {
      // Do not search for any trajectory that takes us off the road.
      if (d(di)<Min_D || d(di)>Max_D) continue;

      for (int ti = 0; ti<T_.size(); ++ti) {
        Vector3d d1;
        d1 << d(di), 0, 0;

        auto coefs = JMT(d0, d1, T_(ti));
        if (!compute_validity(coefs, T_(ti))) continue;

        costs.push_back(compute_cost(coefs,T_(ti), d1-dg, Trajectory(), Mode::LATERAL));
        trajs.push_back(Trajectory(coefs, d1, T_(ti)));
      }
    }
  }

  void following(State const& state0, State const& state_lv, vector<Trajectory>& trajs, vector<double>& costs) const {
    // Get the goal trajectory using the constant distance constant time law
    // Trajectory in x dimension.
    Vector3d slv = state_lv.x;
    slv(2) *= 0.5;

    // Speed in x dimension.
    Vector3d slv_d = polyder(slv, true);

    // Offset
    Vector3d d0;
    d0 << D0, 0, 0;

    Trajectory target(slv - (d0 + tau*slv_d));

    // Compute the trajectories/costs
    following_merging_stoping(state0, target, Mode::FOLLOWING, trajs, costs);
  }

  void merging(State const& state0, State const& state_a, State const& state_b, vector<Trajectory>& trajs, vector<double>& costs) const {
    Vector3d st = (state_a.x + state_b.x)/2;
    st(2) *= 0.5;
    Trajectory target(st);

    // Compute the trajectories/costs
    following_merging_stoping(state0, target, Mode::MERGING, trajs, costs);
  }

  void stopping(State const& state0, double s_end, vector<Trajectory>& trajs, vector<double>& costs) const {
    Vector3d st;
    st << s_end, 0, 0;
    Trajectory target(st);

    // Compute the trajectories/costs
    following_merging_stoping(state0, target, Mode::STOPPING, trajs, costs);
  }

  void velocity_keeping(State const& state0, double sd, ArrayXd const& search_sd, vector<Trajectory>& trajs, vector<double>& costs) const {
    // NOTE! Multiply the final costs by k_lon
  }

  VectorXd JMT(Vector3d s0, Vector3d s1, double T) const {
    return VectorXd::Zero(6);
  }

  VectorXd JMT_vel_keep(Vector3d s0, Vector3d s1, double T) const {
    return VectorXd::Zero(6);
  }

  void following_merging_stoping(State const& state0, Trajectory const& target, Mode const& mode, vector<Trajectory>& trajs, vector<double>& costs) const {
    // Initial state
    Vector3d s0 = state0.x;

    for (int ti = 0; ti<T_.size(); ++ti) {
      // Target state at t.
      Vector3d s1t = target.state_at(T_(ti));

      for (int dsi = 0 ; dsi<ds_.size(); ++dsi) {
        // add in s-coordinate offset
        Vector3d s1 = s1t;
        s1(0) += ds_(dsi);

        auto coefs = JMT(s0, s1, T_(ti));
        if (!compute_validity(coefs, T_(ti))) continue;

        costs.push_back(compute_cost(coefs,T_(ti), s1, target, mode));
        trajs.push_back(Trajectory(coefs, s1, T_(ti)));
      }
    }
  }

  double compute_cost(VectorXd const& coefs, double T, Vector3d const& s1, Trajectory const& target, Mode mode) const {
    // Jerk cost, Cj
    VectorXd p3 = polyder(polyder(polyder(coefs))); // jerk
    VectorXd int_p32 = polyint(polymult(p3,p3)); // integral of jerk^2
    double Cj = kj * polyval(int_p32,T); // jerk cost

    // Time cost
    double Ct = kt * T;

    VectorXd targ = target.ppeval(T);

    // Mode cost
    double Cd;
    switch (mode) {
      case Mode::FOLLOWING:
      case Mode::MERGING:
      case Mode::STOPPING:
        Cd = ks * pow(s1(0) - targ(0),2);
        break;
      case Mode::VELOCITY_KEEPING:
        Cd = ksd * pow(s1(1) - targ(1),2);
        break;
      case Mode::LATERAL:
        Cd = kd * s1(0)*s1(0);
        break;
    }

    // Total cost
    double cost = Cj + Ct + Cd;

    // Multiply by lateral, longitudinal weight
    cost *= (mode == Mode::LATERAL) ? k_lat : k_lon;

    return cost;
  }

  bool compute_validity(VectorXd const& coefs, double T) const {
    return true;
  }

};

#endif
