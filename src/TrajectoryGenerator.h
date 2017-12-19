#ifndef TrajectoryGenerator_H
#define TrajectoryGenerator_H

/* TrajectoryGenerator
A trajectory generation class that is based on the paper "Optimal Trajectory
Generation for Dynamic Street Senarios in the Frenet Frame". (This will be
sited as [1].)
*/


#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <functional>

#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "polynomial.h"
#include "Vehicle.h"
#include "RoadMap.h"

using namespace Eigen;
// using namespace std;

using std::cout;
using std::endl;
using std::vector;

namespace JMTG {

  /* DEBUG level (instead of a logger, which would be nicer)
  999 : no debug
  1 : full debug, some sort of output from most functions
  2 : return lateral trajectories with cost, and longitudinal trajectories with
      cost
  */
  int DEBUG = 999;

  /* Weights for cost function, as defined in the paper [1]*/
  double kj = 1; // weight for the jerk cost
  double kt = 10; // weight fot the time-to-complete cost
  double ks = 100; // weight for the difference in s-coordinate cost
  double ksd = 40; // weight for the difference in s-dot-coordinate cost
  double kd = 2000; // weight for the difference in d-coordinate cost

  double k_lat = 5; // weight for lateral trajectories
  double k_lon = 1; // weight for longitudinal trajectories

  /* Safty_margin - This will be used to ensure the cars do not collide
  or come to close to each other. */
  Vector2d Safty_Margin = (Vector2d()<<1,0.5).finished();

  /* Parameters for constant distance and constant time law. */
  double D0 = 1; // static distance required between cars at speed 0
  double tau = 1; // time distance between cars

  /* Speed, acceleration, and jerk maximum values.
    These should be the highest possible values that the vehicle can function
    at; however, here, they will simply be the maximum values allowed by the
    project.
  */
  double Max_Speed = 22.35;
  double Max_Accel = 10;
  double Max_Jerk = 10;

  /* Information on the road, that really shouldn't be here probably. */
  double Min_D = 0;
  double Max_D = 12;
  double Lane_Width = 4;

  /* Time horizons */
  double Time_Horizon = 2; // used in checking for collisions
  double Reactive_Layer_Time_Horizon = 3;

  /* Properties controling the search values for the new trajectories */
  ArrayXd T_ = ArrayXd::LinSpaced(3,1,3); // times used in searching for new trajectories
  ArrayXd ds_ = ArrayXd::LinSpaced(11,-5,5); // offset longitudinal distances
  ArrayXd dsd_ = ArrayXd::LinSpaced(5,-2,0); // offset speeds
  ArrayXd d_ = ArrayXd::LinSpaced(1,0,0); // offset lateral distances

  /* Function for correcting the lognitudinal distance between to s-coordinates
  Ex. Car 1 has s = s1 and car 2 has s = s2. There difference is s1-s2. Pass
  that differenc to this function apply a custom correction. (For example, if
  the road is periodic, then we want to make sure we are correctly computing
  this difference.)
  The default function is to apply no correction.
   */
  std::function<double(double)> lonDistCorrection = [](double ds) -> double {return ds;};


  namespace INTERNAL {
    VectorXd JMT(Vector3d s0, Vector3d s1, double T) {

      // coef = (s0(0), s0(1), 0.5*s0(2), c(0), c(1), c(2));
      // Must solve for c using calculus of variations, by minimizing the
      // square jerk (3rd derivative)
      //
      // A*c + s0(T) = s1;
      // c = A.inv() *(s1 - s0(T))

      if (DEBUG <= 1) cout << endl <<"----------------------- JMT ----------------------" << endl;
      double T2 = T*T;
      double T3 = T2*T;
      double T4 = T3*T;
      double T5 = T4*T;
      Matrix3d A;
      A <<   T3,    T4,    T5,
           3*T2,  4*T3,  5*T4,
           6*T,  12*T2, 20*T3;

      // Convert the input state to a trajectory.
      s0(2) *= 0.5;
      Vector3d B = s1 - polyeval(s0,T);

      // Solve for c, and return the position polynomial.
      VectorXd coef(6);
      coef << s0, A.inverse()*B;
      if (DEBUG <= 1) cout << "   Polynomial coefs : " << coef.transpose() << endl;
      return coef;
    };

    VectorXd JMT_vel_keep(Vector3d s0, Vector3d s1, double T) {
      // Same problem as JMT, but we dont care about the position, so we can
      // solve for the coefficients of the first derivative, and then integrate
      // to get the position polynomial.

      if (DEBUG <= 1) cout << "-------------------- JMT_vel_keep ----------------" << endl;
      double T2 = T*T;
      double T3 = T2*T;

      Matrix2d A;
      A << T2, T3,
           2*T, 3*T2;

      Vector2d b;
      b << s1(1) - (s0(1) + s0(2)*T), s1(2) - s0(2);

      // Form the polynomial giving the speed.
      Vector4d coef_d;
      coef_d << s0(1), s0(2), A.inverse()*b;

      // Now integrate to get the position polynomial.
      VectorXd coef = polyint(coef_d, s0(0));
      if (DEBUG <= 1) cout << "   Polynomial coefs : " << coef.transpose() << endl;
      return coef;
    };

    bool compute_1d_validity(VectorXd const& coefs, double T, Mode mode) {
      // Determine if a trajectory is valid by ensuring that the speed,
      // acceleration, and jerk are all below the maximum allowed values.
      if (DEBUG <= 1) cout << "--------------- compute_1d_validity ----------------" << endl;
      ArrayXf max_vals(3);
      max_vals << Max_Speed*1.1, Max_Accel, Max_Jerk;
      if (DEBUG <= 1) cout << "   Coefs : " << coefs.transpose() << endl;

      double tol = 1e-10;

      // Get the absolute maximum value of the polynomial in range [0,T]. If the
      // abs(max) value is larger than the max_vals, then the trajectory is not
      // valid; so, return false.
      VectorXd p = polyder(coefs);
      if ((p.array().abs() < tol).all()) return true; // If all values are zero, then it is valid.

      if (mode == Mode::LATERAL) {
        // Check to make sure the car is always within [Min_D, Max_D]
        VectorXd r = realRoots(p);
        double max_val = Min_D;
        double min_val = Max_D;
        for (size_t ri=0 ; ri<r.size(); ++ri) {
          if (r(ri) <0 || r(ri) > T) continue;
          double val = polyval(coefs, r(ri));
          max_val = std::max(max_val, val);
          min_val = std::min(min_val, val);
        }
        double val = polyval(coefs,0);
        max_val = std::max(max_val, val);
        min_val = std::min(min_val, val);
        val = polyval(coefs,T);
        max_val = std::max(max_val, val);
        min_val = std::min(min_val, val);

        if (min_val <= Min_D || max_val >= Max_D) {
          return false;
        }
      }


      if (DEBUG <= 1) cout << "   1st derivative : " << p.transpose() << endl;
      for (size_t j=0; j<3; ++j) {
        VectorXd pp = polyder(p);
        if ((pp.array().abs() < tol).all()) return true; // If all values are zero, then it is valid.

        if (DEBUG <= 1) cout << "   " << j+2 << "th derivative : " << pp.transpose() << endl;
        VectorXd r = realRoots(pp);
        if (DEBUG <= 1) cout << "   Roots of " << j+2 << "th derivative : " << r.transpose() << endl;
        // Maximum value at roots in range [0,T].
        double max_val = 0;
        for (size_t ri=0; ri<r.size(); ++ri) {
          if (r(ri) < 0 || r(ri) > T) continue;
          max_val = std::max(max_val, std::fabs(polyval(p,r(ri))));
        }

        // Maximum value at end points.
        max_val = std::max(max_val, std::fabs(polyval(p,0)));
        max_val = std::max(max_val, std::fabs(polyval(p,T)));

        if (DEBUG <= 1) cout << "   Maximum value of " << j+1 << "th derivative : " << max_val << endl;
        if (max_val > max_vals(j)) {
          if (DEBUG <= 1) cout << "----------------- NOT VALID ----------------------" << endl;
          return false;
        }
        p = pp;
      }

      return true;
    };

    double compute_cost(VectorXd const& coefs, double T, Vector3d const& s1, Trajectory const& target, Mode mode) {

      if (DEBUG <= 1) cout << "----------------- compute_cost -------------------" << endl;
      // Jerk cost, Cj
      VectorXd p3 = polyder(polyder(polyder(coefs))); // jerk
      if (DEBUG <= 1) cout << "   p''' : " << p3.transpose() << endl << endl;
      VectorXd int_p32 = polyint(polymult(p3,p3)); // integral of jerk^2
      if (DEBUG <= 1) cout << "   /  " << endl;
      if (DEBUG <= 1) cout << "   | (p''')^2 dt : " << int_p32.transpose() << endl;
      if (DEBUG <= 1) cout << "   /  " << endl;
      double Cj = kj * polyval(int_p32,T); // jerk cost

      // Time cost
      double Ct = kt * T;

      VectorXd targ = target.ppeval(T);

      // Mode cost
      double Cd = 0;
      switch (mode) {
        case Mode::FOLLOWING:
        case Mode::MERGING:
        case Mode::STOPPING:
          Cd = ks * pow(s1(0) - targ(0),2);
          if (DEBUG <= 1) cout << "   Following/Merging/Stopping cost : " << Cd << endl;
          break;
        case Mode::VELOCITY_KEEPING:
          Cd = ksd * pow(s1(1) - targ(1),2);
          if (DEBUG <= 1) cout << "   Velocity keeping cost : " << Cd << endl;
          break;
        case Mode::LATERAL:
          Cd = kd * pow(fmod(s1(0),Lane_Width),2);
          if (DEBUG <= 1) cout << "   Lateral cost : " << Cd << endl;
          break;
      }

      // Total cost
      double cost = Cj + Ct + Cd;

      // Multiply by lateral, longitudinal weight
      cost *= (mode == Mode::LATERAL) ? k_lat : k_lon;
      if (DEBUG <= 1) cout << "---------- cost = " << cost << " --------------------" << endl;
      return cost;
    };

    void following_merging_stoping(State const& state0, Trajectory const& target, Mode const& mode, vector<Trajectory>& trajs, vector<double>& costs) {
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
          if (!compute_1d_validity(coefs, T_(ti), mode)) continue;

          costs.push_back(compute_cost(coefs,T_(ti), s1, target, mode));
          trajs.push_back(Trajectory(coefs, s1, T_(ti)));
        }
      }
    };

    void lateral(State const& state0, double const& goal_lane, ArrayXd const& search_d, vector<Trajectory>& trajs, vector<double>& costs) {

      if (DEBUG <= 1) cout << "--------------  lateral --------------------------" << endl;
      // Initial lateral state.
      Vector3d d0 = state0.y;

      // The goal state.
      double d_goal = (goal_lane - 0.5)*Lane_Width; // Goal d-coordinate.
      Vector3d dg;
      dg << d_goal, 0, 0;

      if (DEBUG <= 1) cout << "   Goal state : " << dg.transpose() << endl;

      // The search d values.
      ArrayXd d = search_d + d_goal;

      if (DEBUG <= 1) cout << "   Search d-values : " << d.transpose() << endl;

      for (int di =0 ; di<search_d.size(); ++di) {
        // Do not search for any trajectory that takes us off the road.
        if (d(di)<Min_D || d(di)>Max_D) continue;

        for (int ti = 0; ti<T_.size(); ++ti) {
          Vector3d d1;
          d1 << d(di), 0, 0;

          auto coefs = JMT(d0, d1, T_(ti));
          if (!compute_1d_validity(coefs, T_(ti), Mode::LATERAL)) continue;

          costs.push_back(compute_cost(coefs,T_(ti), d1-dg, Trajectory(), Mode::LATERAL));
          trajs.push_back(Trajectory(coefs, d1, T_(ti)));
        }
      }
    };

    void following(State const& state0, State const& state_lv, vector<Trajectory>& trajs, vector<double>& costs) {
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
    };

    void merging(State const& state0, State const& state_a, State const& state_b, vector<Trajectory>& trajs, vector<double>& costs) {
      Vector3d st = (state_a.x + state_b.x)/2;
      st(2) *= 0.5;
      Trajectory target(st);

      // Compute the trajectories/costs
      following_merging_stoping(state0, target, Mode::MERGING, trajs, costs);
    };

    void stopping(State const& state0, double s_end, vector<Trajectory>& trajs, vector<double>& costs) {
      Vector3d st;
      st << s_end, 0, 0;
      Trajectory target(st);

      // Compute the trajectories/costs
      following_merging_stoping(state0, target, Mode::STOPPING, trajs, costs);
    };

    void velocity_keeping(State const& state0, double sd, ArrayXd const& search_sd, vector<Trajectory>& trajs, vector<double>& costs) {
      if (DEBUG <= 1) cout << "----------------- velocity_keeping -----------------" << endl;
      Vector3d s0 = state0.x;
      ArrayXd s_dot = search_sd + sd;

      if (DEBUG <= 1) cout << "   Search s_dot-values : " << s_dot.transpose() << endl;

      // Target trajectory is simply one with a speed of sd.
      Trajectory target((Vector3d()<<0,sd,0).finished());

      if (DEBUG <= 1) {
        cout << "   Goal target : ";
        target.display();
      }

      for (int si =0 ; si<search_sd.size(); ++si) {
        // Do not search for any trajectory that takes us off the road.
        if (s_dot(si)>Max_Speed || s_dot(si) <= 0) continue;

        for (int ti = 0; ti<T_.size(); ++ti) {
          Vector3d s1;
          s1 << 0, s_dot(si), 0;

          auto coefs = JMT_vel_keep(s0, s1, T_(ti));
          if (!compute_1d_validity(coefs, T_(ti), Mode::VELOCITY_KEEPING)) continue;

          s1(0) = polyval(coefs,T_(ti));
          costs.push_back(compute_cost(coefs,T_(ti), s1, target, Mode::VELOCITY_KEEPING));
          trajs.push_back(Trajectory(coefs, s1, T_(ti)));
        }
      }

    };

    bool compute_2d_validity(Vehicle& ego, vector<Vehicle> const& cars, MatrixXd car_traj, VectorXd t, VectorXd min_collision_free_dist, RoadMap const& road){

      if (DEBUG <= 1) cout << "----------------- compute_valididty_2d -----------------" << endl;
      // NOTE: I am using just axis aligned bounding box instead of the exact
      // bounding box. Switch to object aligned bounding box by chaning this flag
      // to true;
      bool OA_DETECTION = true;
      int Nc = cars.size();
      int Nt = t.size();

      // Get ego location/speed, and the map curvature.
      VectorXd ego_s(Nt), ego_d(Nt);
      ArrayXd ego_vs(Nt), ego_vd(Nt), kappa(Nt);
      for (int i=0; i<Nt; ++i) {
        VectorXd s_dat = ego.trajectory[0].ppeval(t(i));
        ego_s(i) = s_dat(0);
        ego_vs(i) = s_dat(1);

        VectorXd d_dat = ego.trajectory[1].ppeval(t(i));
        ego_d(i) = d_dat(0);
        ego_vd(i) = d_dat(1);

        kappa(i) = road.curvature(s_dat(0));
      }

      // Reject trajectrories that have to large of a speed.
      // s^2 = (1-kappa*d)^2*s_dot^2 + d_dot^2
      if ( (((1 - 1.1*kappa *ego_d.array())*ego_vs).square() + ego_vd.square() > 0.995*Max_Speed*Max_Speed).any()) { // the 1.05 is an extra safty factor
        if (DEBUG <= 2) cout << "  Adjust speed over limit!" << endl;
        return true;
      }

      // Collision detection ------------------------------------------------
      // - Get the distance vector between ego and the other cars, and the
      // distance.
      // MatrixXd ego_traj = ego.location_Eig(t);
      MatrixXd dist(Nt,Nc);
      ArrayXXd dloc(Nt*Nc,2);

      for (int ci=0; ci<Nc; ++ci) {
        // Lognitudinal distance using any correction.
        ArrayXd dx = car_traj.block(ci*Nt,0,Nt,1) - ego_s;
        ArrayXd dy = car_traj.block(ci*Nt,1,Nt,1) - ego_d;
        dx = dx.unaryExpr(lonDistCorrection);

        dloc.block(ci*Nt,0,Nt,1) = dx;
        dloc.block(ci*Nt,1,Nt,1) = dy;

        // Normalized distance between cars.
        dist.col(ci) = (dx*dx + dy*dy) / min_collision_free_dist(ci);
      }

      // Find all the points (car/time combinations) that we must check for a collision.
      Matrix<bool,Dynamic,Dynamic> C = dist.array() <= 1;

      if (C.any()) {
        // Get the indices of the points we must check
        VectorXi idx = VectorXi::LinSpaced(C.size(),0,C.size()-1);
        idx.conservativeResize(std::stable_partition(idx.data(), idx.data()+idx.size(), [&C](int i){return C(i);})-idx.data());

        // We have to check all of these points for a collision, however, we can
        // stop checking upon the first detected collision. Therefore, first sort
        // the indices in ascending order of the distance, and start going through
        // the points from the closest. This should help reduce work when there is
        // a collision.
        std::sort(idx.data(), idx.data()+idx.size(), [&dist](size_t i1, size_t i2) {return dist(i1) < dist(i2);});

        for (int ct_i=0; ct_i<idx.size(); ++ct_i) {
          int ti,ci;
          ind2sub(Nt,idx[ct_i],ti,ci);
          // Create the two bounding boxes.
          Rectangle rec1 = ego.bounding_box(t(ti), Vector2d::Zero(2), 2*Safty_Margin);
          Rectangle rec2 = cars[ci].bounding_box(t(ti), dloc.row(ci*Nt+ti), 2*Safty_Margin);
          // Check if they overlap
          if (rec1.overlap(rec2, OA_DETECTION)) {
            if (DEBUG <= 2) cout << "       Collision detected" << endl;
            if (DEBUG <= 1) {

              cout << "        ___________________________" << endl;
              cout << "       | Collision detected" << endl;
              cout << "       |   ego bbox :" << endl;
              cout << "       |      center = 0 0" << endl;
              cout << "       |      x_vert = " << rec1().row(0) << endl;
              cout << "       |      y_vert = " << rec1().row(1) << endl;
              cout << "       |   car bbox :" << endl;
              cout << "       |      center = " << dloc.row(ci*Nt+ti) << endl;
              cout << "       |      x_vert = " << rec2().row(0) << endl;
              cout << "       |      y_vert = " << rec2().row(1) << endl;
              cout << "       |__________________________" << endl;
            }
            return true;
          }
        }
      }
      return false;
    };

    bool generate_(Vehicle& ego, vector<Vehicle> const& cars, SearchMode const& SM, RoadMap const& road, ArrayXd const& search_d, ArrayXd const& search_sd) {

      vector<Trajectory> trajs_d, trajs_s;
      vector<double> costs_d, costs_s;
      State state0 = ego.state;
      vector<Trajectory> traj0 = ego.trajectory;

      // Search for the lateral trajectory
      lateral(state0, SM.goal_lane, search_d, trajs_d, costs_d);
      if (DEBUG <= 1) cout << "Finished lateral ---------------------------------" << endl;
      if (DEBUG <= 1) cout << "   " << trajs_d.size() << " trajectories are valid" << endl;
      // If there were no valid lateral trajectories, then return false
      if (trajs_d.empty()) {
        if (DEBUG <= 1) cout << "   No valid lateral trajectories found!" << endl;
        return false;
      } else {
        // output debug messages
        if (DEBUG <= 2) {
          cout << "--------------------------------------------------" << endl;
          for (int i = 0; i<trajs_d.size(); ++i) {
            cout << "   Lateral trajectory " << i << " : (states) " << endl;
            cout << "      start : " << trajs_d[i].state_at(0).transpose() << endl;
            cout << "      end   : " << trajs_d[i].state_at(trajs_d[i].knot).transpose() << endl;
            cout << "      cost  : " << costs_d[i] << endl;
          }
        }
      }

      // Search for the longitudinal trajectories
      if (DEBUG <= 1) cout << "Starting longitudinal search ---------------------" << endl;
      for (int i=0; i<SM.activeModes.size(); ++i) {
        Mode mode = SM.activeModes[i].mode;
        if (mode == Mode::FOLLOWING ) {
          if (DEBUG <= 1) cout << "   Mode : FOLLOWING" << endl;
          following(state0, SM.activeModes[i].state[0], trajs_s, costs_s);
        } else if (mode == Mode::MERGING ) {
          if (DEBUG <= 1) cout << "   Mode : MERGING" << endl;
          merging(state0, SM.activeModes[i].state[0], SM.activeModes[i].state[1], trajs_s, costs_s);
        } else if (mode == Mode::STOPPING ) {
          if (DEBUG <= 1) cout << "   Mode : STOPPING" << endl;
          stopping(state0, SM.activeModes[i].number, trajs_s, costs_s);
        } else if (mode == Mode::VELOCITY_KEEPING ) {
          if (DEBUG <= 1) cout << "   Mode : VELOCITY_KEEPING" << endl;
          velocity_keeping(state0, SM.activeModes[i].number, search_sd, trajs_s, costs_s);
        }
      }

      if (DEBUG <= 1) cout << "Finished longitudinal ----------------------------" << endl;
      if (DEBUG <= 1) cout << "   " << trajs_s.size() << " trajectories are valid" << endl;
      if (trajs_s.empty()) {
        cout << "No longitudinal trajectories found!" << endl;
        return false;
      } else {
        // output debug messages
        if (DEBUG <= 2) {
          cout << "--------------------------------------------------" << endl;
          for (int i = 0; i<trajs_s.size(); ++i) {
            cout << "   Longitudinal trajectory " << i << " : (states) " << endl;
            cout << "      start : " << trajs_s[i].state_at(0).transpose() << endl;
            cout << "      end   : " << trajs_s[i].state_at(trajs_s[i].knot).transpose() << endl;
            cout << "      cost  : " << costs_s[i] << endl;
          }
        }
      }

      /* Get the linear index of the combination of lateral trajectories with
       longitudinal trajectories with the lowest costs. Ex.

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
        ego.set_trajectory({trajs_s[si],trajs_d[di]});
        return true;

      } else {
        // Check each trajectory combination to see if there are any collisions
        // with the cars, and use the first combination that does not have any
        // collisions.

        if (DEBUG <= 2) cout << "Starting search for best traj without collisions -" << endl;
        int Nt = ceil(Time_Horizon / 0.3) + 1;
        int Nc = cars.size();
        if (DEBUG <= 1) cout << "   Nt = " << Nt << endl;
        if (DEBUG <= 1) cout << "   Nc = " << Nc << endl;
        if (DEBUG <= 1) cout << "   Ns = " << Ns << endl;
        if (DEBUG <= 1) cout << "   Nd = " << Nd << endl;

        // Construct the car trajectories for collision detection and the minimum
        // distance between ego and each car at which there cannot be a collision
        VectorXd t = VectorXd::LinSpaced(Nt,0,Nt-1) * 0.3; // set the times to check for collisions
        MatrixXd car_traj(Nc*Nt, 2); // initialize matrix for storing car paths
        ArrayXd min_collision_free_dist(Nc);

        if (DEBUG <= 1) cout << "Collision detection times : "<< t.transpose() << endl;
        for (int ci=0; ci<Nc; ++ci) {
          // get the car locations
          car_traj.block(ci*Nt,0,Nt,2) << cars[ci].location_Eig(t);

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
          if (DEBUG <= 2) cout << "   Trajectory : di= " << di << ", si=" << si <<", cost = "<< costs_s[si] + costs_d[di] << endl;
          // Set ego's trajectory
          ego.set_trajectory({trajs_s[si], trajs_d[di]});

          // Detect collisions
          collide = compute_2d_validity(ego, cars, car_traj, t, min_collision_free_dist, road);
          if (!collide) break;
        }

        // If all trajectories collide, then set ego's state back to its starting
        // state and return false.
        if (collide) {
          // ego.set_state(state0);
          ego.set_trajectory(traj0);
          return false;
        }
      }

      return true;
    };
  };

  bool reactive(Vehicle& ego, vector<Vehicle> const& cars, RoadMap const& road) {

    double v_ref = Max_Speed * 0.97;
    // Get mean car speed.
    if (!cars.empty()) {
      double mean_speed = 0;
      for (auto& v : cars) mean_speed += v.state.x(1);
      mean_speed /= cars.size();

      v_ref = std::min(mean_speed + 5, v_ref);
    }

    ActiveMode aM(Mode::VELOCITY_KEEPING, v_ref);
    SearchMode sM(aM, 2);


    ArrayXd sd_search(12); // = ArrayXd::LinSpaced(12,-20,2);
    sd_search << -15, -12.5, -10, -7.5, -6, -4.5, -3, -2, -1, -0.5, 0, 1;
    ArrayXd d_search(3);// = ArrayXd::LinSpaced(7,-6,6);
    d_search << -4,0,4;//

    return INTERNAL::generate_(ego, cars, sM, road, d_search, sd_search);
  };

  /* Generate the best trajectory for ego using the SearchMode while preventing
  collisions with the cars. If no trajectory is found, then false is returned;
  otherwise, true is returned.
  */
  bool generate(Vehicle& ego, vector<Vehicle> const& cars, SearchMode const& SM, RoadMap const& road) {
    return INTERNAL::generate_(ego, cars, SM, road, d_, dsd_);
  };
};

#endif
