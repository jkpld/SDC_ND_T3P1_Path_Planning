#ifndef Behavior_H
#define Behavior_H

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "Vehicle.h"
#include "TrajectoryGenerator.h"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

struct BehaviorData_t {
  SearchMode SM;
  int N;
  bool attempt_switch;

  BehaviorData_t (SearchMode SM, bool aS, int N) : SM(SM), N(N), attempt_switch(aS) {};
  BehaviorData_t() : SM(SearchMode()), N(999), attempt_switch(false) {};
};

class BehaviorModule {
public:
  double Behavior_Horizon;
  double Replanning_Time;
  double Consider_Distance;
  double Goal_Speed;
  double Min_Tight_Merge_Offset;
  double Max_Tight_Merge_Offset;

  RoadMap road;

private:
  double ego_Length;
  double ego_Width;
  int ego_lane;
  double ego_speed;
  double v_ref;
  int Goal_Lane;

  double lane_width;

public:

  BehaviorModule(Vehicle& ego, double BH, double RT, double CD, double GS, RoadMap& RM) :
    Behavior_Horizon(BH),
    Replanning_Time(RT),
    Consider_Distance(CD),
    Goal_Speed(GS),
    Min_Tight_Merge_Offset(2),
    Max_Tight_Merge_Offset(20),
    road(RM),
    ego_Length(ego.get_length()),
    ego_Width(ego.get_width()),
    ego_lane(-1),
    ego_speed(0),
    v_ref(0),
    Goal_Lane(-1) {
      JMTG::lonDistCorrection = road.lonDistCorrection;
      lane_width = road.lane_width;
    };

  void display() {
    cout << "Behavior Module with properties:" << endl;
    cout << "  Behavior_Horizon = " << Behavior_Horizon << endl;
    cout << "  Replanning_Time = " << Replanning_Time << endl;
    cout << "  Consider_Distance = " << Consider_Distance << endl;
    cout << "  Goal_Speed = " << Goal_Speed << endl;
    cout << "  Min_Tight_Merge_Offset = " << Min_Tight_Merge_Offset << endl;
    cout << "  Max_Tight_Merge_Offset = " << Max_Tight_Merge_Offset << endl;
    cout << "  Goal_Lane = " << Goal_Lane << endl;
    cout << "  ego_Length = " << ego_Length << endl;
    cout << "  ego_Width = " << ego_Width << endl;
  }

  bool PlanPath(Vehicle& ego, vector<Vehicle> const& all_cars) {

    ego_lane = get_lane(ego.state.y(0));
    ego_speed = ego.state.x(1);
    v_ref = std::min(ego_speed+10, Goal_Speed);

    if (Goal_Lane < 0) {
      Goal_Lane = ego_lane;
    }

    int Nc = all_cars.size();

    /* Create set of SearchModes to use */
    vector<SearchMode> SMs;
    vector<Vehicle> cars;
    vector<int> lane;
    vector<double> speed;
    vector<Vector2d> ds;

    if (Nc>0) {
      /* Get cars that our near our location now or in @Behavior_Horizon */
      Vector2d t01;
      t01 << 0, Behavior_Horizon;
      Matrix2d ego_loc = ego.location_Eig(t01);


      for (auto& v : all_cars) {
        Matrix2d car_loc = v.location_Eig(t01);
        Vector2d dist_s = car_loc.col(0) - ego_loc.col(0);
        dist_s = dist_s.unaryExpr(road.lonDistCorrection);

        if ((dist_s.array().abs() < Consider_Distance*2).any()) {
          cars.push_back(v);
          lane.push_back(get_lane(car_loc(0,1)));
          speed.push_back(v.state.x(1) + 0.5*v.state.x(2)*Behavior_Horizon);// average speed (s_dot) between now and at Behavior_Horizon.
          ds.push_back(dist_s);
        }
      }
    }

    Nc = cars.size();
    if (Nc==0) {
      // If there are no cars the, stay in the same lane.
      SMs.push_back(SearchMode(ActiveMode(Mode::VELOCITY_KEEPING, v_ref), ego_lane));
    } else {

      vector<size_t> idx(cars.size());
      iota(idx.begin(), idx.end(), 0);

      vector<int> Nf(road.num_lanes+2,999), Nb(road.num_lanes+2,999), N(road.num_lanes+2,999);
      for (int k=1; k<4; ++k) {
        Nf[k] = std::accumulate(idx.begin(), idx.end(), 0, [&](int i, int j){return i+(lane[j]==k && ds[j](0)>-ego_Length*(j!=Goal_Lane));});
        Nb[k] = std::accumulate(idx.begin(), idx.end(), 0, [&](int i, int j){return i+(lane[j]==k && ds[j](0)<=-ego_Length*(j!=Goal_Lane));});
        N[k] = Nf[k] + Nb[k];
      }

      cout << "Cars in front in lane 1/2/3 : " << Nf[1] << " / " << Nf[2] << " / " << Nf[3] << endl;
      cout << "Cars in back in lane 1/2/3 : " << Nb[1] << " / " << Nb[2] << " / " << Nb[3] << endl;
      cout << "Cars in front of ego : " << N[ego_lane] << endl;

      if (Nf[ego_lane] == 0) {
        SMs.push_back(SearchMode(ActiveMode(Mode::VELOCITY_KEEPING, v_ref), ego_lane));
      } else {
        BehaviorData_t center_dat = Search_Center(ds, speed, lane, cars);
        SMs.push_back(center_dat.SM);

        if (center_dat.attempt_switch) {
          if (Nf[Goal_Lane-1] == 0) {
            BehaviorData_t dat = Search_LeftRight(-1, ds, speed, lane,  cars);
            SMs.push_back(dat.SM);
          } else if (Nf[Goal_Lane+1] == 0) {
            BehaviorData_t dat = Search_LeftRight(1, ds, speed, lane,  cars);
            SMs.push_back(dat.SM);
          } else {
            for (int i=-2; i<3; ++i) {
              if ((Goal_Lane+i < 1) || (Goal_Lane+i > road.num_lanes) || (i==0)) continue;
              BehaviorData_t L_dat = Search_LeftRight(i, ds, speed, lane,  cars);
              BehaviorData_t R_dat = Search_LeftRight(i, ds, speed, lane,  cars);
              SMs.push_back(L_dat.SM);
              SMs.push_back(R_dat.SM);
            }
          }
        }
      }
    }

    bool all_collide = true;
    double min_cost = 1e300;
    Vehicle best_ego = ego;
    SearchMode best_SM;
    Vehicle ego_copy = ego;
    // cout << "EGO's state:" << endl;
    // ego.state.display();
    for (auto& sm : SMs) {
      sm.display();
      bool success_i = JMTG::generate(ego_copy, cars, sm, road);
      // if (success_i) {
      //   cout << "Success!" << endl;
      // } else {
      //   cout << "Collision!" << endl;
      // }
      double cost_i = compute_cost(ego_copy, sm);
      cost_i += (!success_i) ? 1e300 : 0;

      if (cost_i < min_cost) {
        min_cost = cost_i;
        best_ego = ego_copy;
        best_SM = sm;
      }

      all_collide = all_collide && !success_i;
      ego_copy = ego;
    }

    if (all_collide) {
      cout << "::::::::::::::::::::::::::::::::::::" << endl;
      cout << "::::::::: REACTIVE LAYER :::::::::::" << endl;
      cout << "::::::::::::::::::::::::::::::::::::" << endl;
      bool collide = JMTG::reactive(ego, cars, road);
      State s1 = ego.state_at(100);
      Goal_Lane = 1 + (s1.y(0)/road.lane_width);
      return collide;
    } else {
      ego = best_ego;
      Goal_Lane = best_SM.goal_lane;
      return true;
    }
  }

  double compute_cost(Vehicle const& ego, SearchMode const& SM) {
    State s1 = ego.state_at(Behavior_Horizon);

    double cost = 0;
    cost += 2*(1-s1.x(1)/Goal_Speed);

    cost += (SM.min_lv_speed>0) ? 2*(1-SM.min_lv_speed/Goal_Speed) : 0;
    cost += 0.2*abs(SM.goal_lane - Goal_Lane);
    return cost;
  }

  BehaviorData_t Search_LeftRight(int L_or_R, vector<Vector2d> ds01, vector<double> speed, vector<int> lane, vector<Vehicle> cars) {
    // if (L_or_R==-1) cout << "---------------- Search Left --------------------" << endl;
    // else cout << "---------------- Search Right -------------------" << endl;

    SearchMode SM(ActiveMode(Mode::VELOCITY_KEEPING, v_ref), Goal_Lane + L_or_R);


    // cars in lane
    cars_in_lane(Goal_Lane + L_or_R, ds01, speed, lane, cars);

    if (L_or_R < 0) cout << "Cars in left : " << cars.size() << endl;
    else cout << "Cars in right : " << cars.size() << endl;

    if (cars.empty()) return BehaviorData_t(SM, 0, false);

    int Nc = cars.size();

    if (Nc == 1) {
      if ((ds01[0].array()>0).any()) {
        SM.activeModes.push_back(ActiveMode(Mode::FOLLOWING, {cars[0].state}));
        if ((ds01[0].array()<Consider_Distance).any()) SM.min_lv_speed = speed[0];
        return BehaviorData_t(SM, Nc, false);
      }
    } else {

      // sort from farthest to closest
      VectorXi idx = VectorXi::LinSpaced(cars.size(),0,cars.size()-1);
      std::sort(idx.data(), idx.data()+idx.size(), [&ds01](int i1, int i2){return ds01[i1](0) < ds01[i2](0);});
      ds01 = select_index(ds01,idx);
      speed = select_index(speed,idx);
      cars = select_index(cars,idx);


      // minimum distance between cars now and at Behavior_Horizon
      vector<double> ds;
      for (int i=0; i<Nc-1; ++i) {
        ds.push_back(std::min(ds01[i+1](0) - ds01[i](0), ds01[i+1](1) - ds01[i](1)));
      }

      // if the car farthest back is going faster than us, then add a following mode for it.
      if (speed[0] > ego_speed) {
        SM.activeModes.push_back(ActiveMode(Mode::FOLLOWING, {cars[0].state}));
      }

      for (int i=0; i<Nc-1; ++i) {
        double free_space = ds[i] - cars[i].get_length()/2 - cars[i+1].get_length()/2 - ego_Length;
        if (free_space > Max_Tight_Merge_Offset) {
          SM.activeModes.push_back(ActiveMode(Mode::FOLLOWING, {cars[i+1].state}));
        } else if (free_space > Min_Tight_Merge_Offset) {
          SM.activeModes.push_back(ActiveMode(Mode::MERGING, {cars[i].state, cars[i+1].state}));
        }
      }

      // get the minimum car speed in front of us
      idx = VectorXi::LinSpaced(cars.size(),0,cars.size()-1);
      idx.conservativeResize(std::stable_partition(idx.data(), idx.data()+idx.size(), [&ds01](int i){return (ds01[i](0) >0);})-idx.data());
      speed = select_index(speed,idx);
      ds01 = select_index(ds01,idx);
      if (!speed.empty()) {
        if ((*std::min_element(ds01.begin(), ds01.end(), [&](Vector2d i1, Vector2d i2){return i1(0)<i2(0);}))(0) < Consider_Distance)
          SM.min_lv_speed = *std::min_element(speed.begin(), speed.end());
      }
    }

    return BehaviorData_t(SM, Nc, false);
  }

  BehaviorData_t Search_Center(vector<Vector2d> ds01, vector<double> speed, vector<int> lane, vector<Vehicle> cars) {

    // cars in lane
    cars_in_lane(Goal_Lane, ds01, speed, lane, cars);
    cout << "Cars in center : " << cars.size() << endl;
    if (cars.empty()) {
      SearchMode SM(ActiveMode(Mode::VELOCITY_KEEPING, v_ref), Goal_Lane);
      return BehaviorData_t(SM, 0, false);
    }

    SearchMode SM;
    SM.goal_lane = Goal_Lane;
    // get the minimum car speed in front of us
    // VectorXi idx = VectorXi::LinSpaced(cars.size(),0,cars.size()-1);
    // idx.conservativeResize(std::stable_partition(idx.data(), idx.data()+idx.size(), [&ds01](int i){return (ds01[i](0) >0);})-idx.data());
    // speed = select_index(speed,idx);
    // if (!speed.empty()) {
    //   SM.min_lv_speed = *std::min_element(speed.begin(), speed.end());
    // }
    SM.min_lv_speed = -1;

    double nf = 1e200;
    int nfi = -1;
    double nb = -1e200;
    double nbi = -1;
    for (int i=0; i<cars.size(); ++i) {
      if (ds01[i](0) > 0 && ds01[i](0) < nf) {
        nf = ds01[i](0);
        nfi = i;
      } else if (ds01[i](0) < 0 && ds01[i](0) > nb) {
        nb = ds01[i](0);
        nbi = i;
      }
    }

    bool attempt_switch = false;
    if (nfi != -1) {
      // there is a car infront
      if (nbi != -1) {
        // there is also a car in back
        double free_space = (nf-nb) - cars[nbi].get_length()/2 - cars[nfi].get_length()/2 - ego_Length;
        if (free_space > Max_Tight_Merge_Offset) {
          if (nf < Consider_Distance)
            SM.activeModes = {ActiveMode(Mode::FOLLOWING, {cars[nfi].state})};
          else
            SM.activeModes = {ActiveMode(Mode::VELOCITY_KEEPING, v_ref)};
        } else if (free_space > Min_Tight_Merge_Offset) {
          SM.activeModes = {ActiveMode(Mode::MERGING, {cars[nbi].state, cars[nfi].state})};
        }
      } else {
        if (nf < Consider_Distance)
          SM.activeModes = {ActiveMode(Mode::FOLLOWING, {cars[nfi].state})};
        else
          SM.activeModes = {ActiveMode(Mode::VELOCITY_KEEPING, v_ref)};
      }

      attempt_switch = speed[nfi] < v_ref;
    } else {
      SM.activeModes = {ActiveMode(Mode::VELOCITY_KEEPING, v_ref)};
    }

    return BehaviorData_t(SM, cars.size(), attempt_switch);
  }

  void cars_in_lane(int lane_num, vector<Vector2d>& ds, vector<double>& speed, vector<int> const& lane, vector<Vehicle>& cars) {
    if (cars.size()>0) {
      // cout << " Retrieving cars in lane" << endl;
      // cout << "   Ns = " << speed.size() << ", Nl = " << lane.size() << ", Nc = " << cars.size() << ", lane_num = " << lane_num <<endl;
      VectorXi idx = VectorXi::LinSpaced(lane.size(),0,lane.size()-1);
      idx.conservativeResize(std::stable_partition(idx.data(), idx.data()+idx.size(), [&](int i){return (lane[i] == lane_num);})-idx.data());

      ds = select_index(ds,idx);
      speed = select_index(speed,idx);
      cars = select_index(cars,idx);

      cout << " Cars in lane " << lane_num << " (" << cars.size() << ") : ";
      for (int i=0; i<cars.size(); ++i) {
        cout << cars[i].state.y(0) << " ";
      }
      cout << endl;
    }
  }

  int get_lane(double s) {
    return 1 + (s/lane_width);
    if (s <= 3 && s>= 1) return 1;
    if (s <= 7 && s>= 5) return 2;
    if (s <= 11 && s >= 9) return 3;
  }

};


#endif
