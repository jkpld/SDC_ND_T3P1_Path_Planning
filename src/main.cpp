#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <cassert>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "BehaviorModule.h"
#include "Vehicle.h"
#include "RoadMap.h"

using namespace std;

// for convenience
using json = nlohmann::json;

double TIME_STEP = 0.02;
double REPLANNING_TIME = 0.5;
double PLAN_HORIZON = 2.5;
double CONSIDER_DISTANCE = 30;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

template <typename T>
T dist2(T x1, T y1, T x2, T y2){
  T dx = x1-x2;
  T dy = y1-y2;
  return dx*dx + dy*dy;
};

// Functions for timing
chrono::high_resolution_clock::time_point start_timer() {
    return chrono::high_resolution_clock::now();
}
double stop_timer (chrono::high_resolution_clock::time_point &start) {
    double duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start).count();
    return duration;
};

int main() {
  uWS::Hub h;

  // Initialize road
  RoadMap road("../data/highway_map.csv");

  map<int, Vehicle> cars;
  Vehicle ego(0);
  vector<Trajectory> end_traj;

  // Update JMTG lonDistCorrection function to be the same as our road's (which,
  // makes the road periodic.).
  double goal_speed = 20.2;
  // CONSIDER_DISTANCE = goal_speed * PLAN_HORIZON;
  BehaviorModule behavior(ego, PLAN_HORIZON, REPLANNING_TIME, CONSIDER_DISTANCE, goal_speed, road);

  auto start_time = start_timer();

  h.onMessage([&road, &ego, &cars, &behavior, &end_traj, &start_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {

          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          vector<double> previous_path_x = j[1]["previous_path_x"];
        	vector<double> previous_path_y = j[1]["previous_path_y"];
          auto sensor_fusion = j[1]["sensor_fusion"];

          int N = previous_path_x.size();
          double t_end = N * TIME_STEP;
          double t = stop_timer(start_time);

          // cout << "   s = " << car_s << " , d = " << car_d << "  :   N_sensor = " << sensor_fusion.size()<< endl;

          // map<int, Vehicle> cars;

          // Update all cars
          for (auto& car : sensor_fusion) {
            // Create the car if not seed before.
            int id = car[0];
            if (cars.count(id) == 0) cars[id] = Vehicle(id); // create the car

            double s = car[5];
            double d = car[6];

            double kappa = road.curvature(s);
            // Add the cars measurment - in frenet
            // NOTE: velocity transofrm is missing factor with curvature.
            Matrix2d M = road.TransformMat_C2F(s);
            Vector2d v = M * (Vector2d() << car[3], car[4]).finished();
            cars[car[0]].new_measurment({s,d,v(0)/(1-kappa*d),v(1)}, t); // the car already has been added
          }

          vector<double> path_x;
          vector<double> path_y;

          if (t_end < PLAN_HORIZON) {

            // Get ego's last known trajectory (if any)
            if (t_end == 0) {
              // Initialize ego's location if first call
            	double car_yaw = j[1]["yaw"];
            	double car_speed = j[1]["speed"];

              // Get the transformation matrices between frenet and cartesian
              Matrix2d M = road.TransformMat_C2F(car_s);

              // Get the velocity vector in cartesian
              // NOTE: velocity transofrm is missing factor with curvature.
              car_yaw = deg2rad(car_yaw);
              Vector2d v_xy = (Vector2d()<< cos(car_yaw), sin(car_yaw)).finished() * car_speed * 0.44704; // [m/s]
              Vector2d v_sd = M * v_xy; // Convert to Frenet
              double kappa = road.curvature(car_s);

              /* Get ego's trajecotry. Note, in this case the state (position,
              speed, acceleration), are the coefficients of the trajectory,
              since the acceleration is assumed 0. */
              Vector3d s_state = (Vector3d()<< car_s, v_sd(0)/(1-kappa*car_d), 0).finished();
              Vector3d d_state = (Vector3d()<< car_d, v_sd(1), 0).finished();
              ego.set_trajectory({Trajectory(s_state), Trajectory(d_state)});

            } else {
              ego.set_trajectory(end_traj);
            }
            ego.t0 = 0;

            // Move all cars forward in time to correspond to ego.
            vector<Vehicle> near_cars;
            for (auto& v : cars) {
              Vehicle car = v.second;
              car.set_state(car.state_at(REPLANNING_TIME));
              car.t0 = 0;
              near_cars.push_back(car);
            }

            /* Predict a new trajectory for ego.
            At this time, just use reactive-trajectory generation and skip
            the behavioral planning parts. This will likely lead to recless
            driving, but it should hopefully pass the project. */
            auto tic = start_timer();
            // bool success = JMTG::reactive(ego, near_cars, road);
            cout << "---------------- Start Planning -----------------" << endl;
            bool success = behavior.PlanPath(ego, near_cars);
            auto toc = stop_timer(tic);
            cout << "Successful trajectory generation? " << success << " (" << t << "/" << toc << ")" << endl;

            // ego.display();

            // car location in frenet at t = 0 (which is t=t_end);
            vector<double> loc_sd = ego.location(0);
            auto xy = road.getXY(loc_sd[0], loc_sd[1]);

            // Our new path and our old path should cover approximately the same
            // REPLANNING_TIME of distance. So, we need to find our where the
            // old path and the new path connect, and then replance the old path
            // with the new path.
            // -- find out which element in old-path is closest to the first
            // -- element of the new path
            // cout << "searching for overlap location" << endl;

            size_t idx0 = 0;
            if (N > 0) {
              vector<size_t> idx(N);
              iota(idx.begin(), idx.end(), 0);

              std::sort(idx.begin(), idx.end(), [&](size_t i1, size_t i2){
                return dist2(previous_path_x[i1], previous_path_y[i1], xy[0], xy[1]) <
                  dist2(previous_path_x[i2], previous_path_y[i2], xy[0], xy[1]);
              });

              // if (dist2(previous_path_x[idx[0]],previous_path_y[idx[0]],xy[0],xy[1]) > 1)
                // assert(1==2);

              idx0 = idx[0];
            }

            // Construct the new path points.
            // - first get all previous points for idx < idx[0]
            for (size_t i=0; i<idx0; ++i) {
              path_x.push_back(previous_path_x[i]);
              path_y.push_back(previous_path_y[i]);
            }

            // - now add on the newly computed path.
            for (double ti = 0; ti <= PLAN_HORIZON; ti += TIME_STEP) {

              // car location in frenet at t = ti;
              vector<double> loc_sd = ego.location(ti);

              // car location in cartesian
              auto xy = road.getXY(loc_sd[0], loc_sd[1]);
              path_x.push_back(xy[0]);
              path_y.push_back(xy[1]);
              // cout << xy[0] << "\t" << xy[1] << endl;
            }

            end_traj = ego.trajectory_at(REPLANNING_TIME);
            if (end_traj[0].coef[0](0) > road.RoadLength) {
              end_traj[0].coef[0](0) -= road.RoadLength;
            }
          } else {
            path_x = previous_path_x;
            path_y = previous_path_y;


          }

          // assert(2==0);

          /* Create commands to send to simulator */
          json msgJson;
        	msgJson["next_x"] = path_x;
        	msgJson["next_y"] = path_y;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	// this_thread::sleep_for(chrono::milliseconds(200));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
