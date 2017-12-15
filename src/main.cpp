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
#include "TrajectoryGenerator.h"
#include "Vehicle.h"
#include "RoadMap.h"

using namespace std;

// for convenience
using json = nlohmann::json;

double TIME_STEP = 0.02;
double REPLANNING_TIME = 0.6;
double PLAN_HORIZON = 1;
double CONSIDER_DISTANCE = 50;

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

  auto start_time = start_timer();

  h.onMessage([&road, &cars, &ego, &end_traj, &start_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double t_end = previous_path_x.size() * TIME_STEP;
          double t = stop_timer(start_time);

          // cout << "Current time remaining: " << t_end << endl;
          cout << "   s = " << car_s << endl;
          // cout << "   d = " << car_d << endl;
          // cout << "   x = " << car_x << endl;
          // cout << "   y = " << car_y << endl;
          // cout << endl;

          // vector<double> next_x_vals;
          // vector<double> next_y_vals;
          //
          // for (int i=0; i<previous_path_x.size(); ++i) {
          //   next_x_vals.push_back(previous_path_x[i]);
          //   next_y_vals.push_back(previous_path_y[i]);
          // }

          // Update all cars
          for (auto& car : sensor_fusion) {
            // Create the car if not seed before.
            int id = car[0];
            if (cars.count(id) == 0) cars[id] = Vehicle(id); // create the car

            // Add the cars measurment - in frenet
            Matrix2d M = road.TransformMat_C2F(car[5]);
            Vector2d v = M * (Vector2d() << car[3], car[4]).finished();
            cars[car[0]].new_measurment({car[5],car[6],v(0),v(1)}, t); // the car already has been added
          }

          if (t_end < REPLANNING_TIME) {

            // Get ego's last known trajectory (if any)
            if (t_end == 0) {
              // Initialize ego's location if first call
            	double car_yaw = j[1]["yaw"];
            	double car_speed = j[1]["speed"];

              // Get the transformation matrices between frenet and cartesian
              Matrix2d M = road.TransformMat_C2F(car_s);

              // Get the velocity vector in cartesian
              car_yaw = deg2rad(car_yaw);
              Vector2d v_xy = (Vector2d()<< cos(car_yaw), sin(car_yaw)).finished() * car_speed * 0.44704; // [m/s]
              Vector2d v_sd = M * v_xy; // Convert to Frenet

              /* Get ego's trajecotry. Note, in this case the state (position,
              speed, acceleration), are the coefficients of the trajectory,
              since the acceleration is assumed 0. */
              Vector3d s_state = (Vector3d()<< car_s, v_sd(0), 0).finished();
              Vector3d d_state = (Vector3d()<< car_d, v_sd(1), 0).finished();
              ego.set_trajectory({Trajectory(s_state), Trajectory(d_state)});

              // cout << "Initializing ego ..." << endl;
              // ego.display();
              // cout << endl;
            } else {
              ego.set_trajectory(end_traj);
              // cout << "Using last known ego location ..." << endl;
              // ego.display();
              // cout << endl;
            }
            ego.t0 = 0;



            vector<double> ego_sd0 = {car_s, car_d};
            vector<double> ego_sd1 = ego.location(0);
            vector<double> ego_sd2 = ego.location(PLAN_HORIZON);

            /* Predict car locations at t_end and at t_end + PLAN_HORIZON. Any
            car within CONSIDER_DISTANCE any any of the three times will be
            used when generating trajectories for collision avoidance. */
            vector<Vehicle> near_cars;
            for (auto& v : cars) {
              vector<double> sd0 = v.second.location(0);
              vector<double> sd1 = v.second.location(t_end);
              vector<double> sd2 = v.second.location(t_end + PLAN_HORIZON);

              if (fabs(sd0[0] - ego_sd0[0]) < CONSIDER_DISTANCE ||
                  fabs(sd1[0] - ego_sd1[0]) < CONSIDER_DISTANCE ||
                  fabs(sd2[0] - ego_sd2[0]) < CONSIDER_DISTANCE) {

                // Move the car forward in time to t_end, and add it to our
                // near cars.
                Vehicle car = v.second;
                car.set_state(car.state_at(t_end));
                car.t0 = 0;
                near_cars.push_back(car);
              }
            }

            /* Predict a new trajectory for ego.
            At this time, just use reactive-trajectory generation and skip
            the behavioral planning parts. This will likely lead to recless
            driving, but it should hopefully pass the project. */
            bool success = JMTG::reactive(ego, near_cars);
            cout << "Successful trajectory generation? " << success << endl;

            // ego.display();


            // compute new location points to give to simulator
            for (double ti = 0; ti <= PLAN_HORIZON; ti += TIME_STEP) {

              // car location in frenet at t = ti;
              vector<double> loc_sd = ego.location(ti);

              // car location in cartesian
              auto xy = road.getXY(fmod(loc_sd[0], RoadLength), loc_sd[1]);
              previous_path_x.push_back(xy[0]);
              previous_path_y.push_back(xy[1]);
              // cout << xy[0] << "\t" << xy[1] << endl;
            }

            end_traj = ego.trajectory_at(PLAN_HORIZON);
          }



          // next_x_vals.clear();
          // next_y_vals.clear();
          // for (int i = 0; i< 50; ++i) {
          //   auto xy = road.getXY(car_s+i, 6);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          // }

          // for (int i=0; i<next_x_vals.size(); ++i) {
          //   cout << next_x_vals[i] << "\t" << next_y_vals[i] << endl;
          // }

          // assert(2==0);

          /* Create commands to send to simulator */
          json msgJson;
        	msgJson["next_x"] = previous_path_x;
        	msgJson["next_y"] = previous_path_y;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
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
