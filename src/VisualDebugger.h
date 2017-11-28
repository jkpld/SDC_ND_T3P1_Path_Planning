#ifndef VisualDebugger_H
#define VisualDebugger_H

#include "matplotlibcpp/matplotlibcpp.h"
#include "Vehicle.h"
#include "RoadMap.h"
#include <vector>
#include <iostream>

using namespace std;
namespace plt = matplotlibcpp;

class VisualDebugger {
private:
  bool first_call = true;
public:

  enum CordSystems { CARTESIAN, FRENET } CordSys;

  VisualDebugger() {CordSys = VisualDebugger::CARTESIAN;};
  virtual ~VisualDebugger() {};

  void update(Vehicle ego, map<int,Vehicle>& cars, RoadMap& road) {
    if (first_call) {
      first_call = false;
    } else {
      plt::clf();
    }

    draw_car(ego, road, true);

    for (auto& i : ego.neighborhood) {
      draw_car(cars[i], road, false, i);
    }

    // set axis limits
    if (CordSys == VisualDebugger::CARTESIAN) {
      auto ego_sf = ego.state;
      plt::xlim(-50+ego_sf[1],50+ego_sf[1]);
      plt::ylim(-50+ego_sf[2],50+ego_sf[2]);
    } else {

      auto ego_sf = road.Cartesian_to_Frenet(ego.state);
      plt::ylim(-3*road.lane_width,7*road.lane_width);
      plt::xlim(-50+ego_sf[1],50+ego_sf[1]);
    }

    draw_road(road);
    plt::pause(0.0001);
  }

  void draw_road(RoadMap& road) {
    if (CordSys == VisualDebugger::CARTESIAN) {

      auto wp = road.get_waypoints();
      MatrixXd wp_r = wp[0];
      MatrixXd wp_n = wp[1];
      for (int i = 0; i<4; i++) {
        auto line = wp_r + double(i)*road.lane_width*wp_n;
        vector<double> vx;
        vector<double> vy;
        for (int j=0; j<wp_r.rows(); j++) {
          vx.push_back(line(j,0));
          vy.push_back(line(j,1));
        }


        string mark = (i==0 || i==3) ? ((i==0) ? "y-" : "k-") : "k--";

        plt::plot(vx,vy,mark);
      }

    } else {
      auto xlim = plt::xlim();
      auto ylim = plt::ylim();

      auto lw = road.lane_width;

      // draw lanes
      plt::plot({xlim[0],xlim[1]},{0,0},"y-");
      plt::plot({xlim[0],xlim[1]},{-0.2,-0.2},"y-");
      plt::plot({xlim[0],xlim[1]},{lw,lw},"k--");
      plt::plot({xlim[0],xlim[1]},{2*lw,2*lw},"k--");
      plt::plot({xlim[0],xlim[1]},{3*lw,3*lw},"k-");
    }
  };

  void draw_car(Vehicle& car, RoadMap& road, bool is_ego, int id = 0) {

    auto s = car.state;

    if (CordSys == VisualDebugger::FRENET) {
      s = road.Cartesian_to_Frenet(car.state);
    }

    string mark = (is_ego) ? "ro" : "bs";
    plt::plot({s[1]},{s[2]},mark);
    plt::plot({s[1],s[1]+s[3]/2}, {s[2],s[2]+s[4]/2}, "b-");
    plt::plot({s[1],s[1]+s[5]/5}, {s[2],s[2]+s[6]/5}, "k-");

    double w = 1;
    double h = 2.4;

    mark = (is_ego) ? "r-" : "b-";

    vector<double> vrt_x = {-w, -w, w, w, -w};
    vector<double> vrt_y = {-h, h, h, -h, -h};

    double theta = atan2(s[4],s[3]);
    for (int i = 0; i<5; i++) {
      auto tx = vrt_x[i];
      auto ty = vrt_y[i];
      vrt_x[i] = tx*cos(theta) + ty*sin(theta) + s[2];
      vrt_y[i] = -tx*sin(theta) + ty*cos(theta) + s[1];
    }

    plt::plot( vrt_y, vrt_x, mark);

    if (is_ego) {
      plt::annotate("ego", s[1],s[2]);
    } else {
      plt::annotate(to_string(id), s[1],s[2]);
    }

    // Compute te cars predicted trajectory in Frenet coordinates
    s = road.Cartesian_to_Frenet(car.state);
    auto traj = car.generate_predicted_trajectory(s);

    vector<double> traj_x;
    vector<double> traj_y;
    for (auto& tf : traj) {
      auto t = tf;
      if (CordSys == VisualDebugger::CARTESIAN) {
        t = road.Frenet_to_Cartesian(tf);
      }

      traj_x.push_back(t[2]);
      traj_y.push_back(t[1]);
    }

    plt::plot(traj_y, traj_x, "g-");
  };
};

#endif
