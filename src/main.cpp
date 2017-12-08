#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "Vehicle.h"
#include <map>
#include "RoadMap.h"
#include "VisualDebugger.h"
#include "PTG.h"
#include "Trajectory.h"
// #include "TrajectoryCost.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

VisualDebugger plotter;

int main() {
  uWS::Hub h;

  // Initialize road
  RoadMap road = RoadMap("../data/highway_map.csv");
  plotter.CordSys = VisualDebugger::CARTESIAN;

  map<int, Vehicle> cars;
  auto startTime = start_timer();
  Vehicle ego = Vehicle(road, 0);
  Vehicle target = Vehicle(road);
  vector<double> end_state(7,0);
  PTG ptg;

  bool first_call = true;

  h.onMessage([&road, &cars, &ego, &startTime, &ptg, &end_state, &target, &first_call](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          /* --------------------------------------------------------------- */
        	/* Update main car's state */
          /* --------------------------------------------------------------- */
          if (first_call) {
            startTime = start_timer();
            // first_call = false;
          }


        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];
          double t0 = stop_timer(startTime);

          car_speed = car_speed*0.44704; // convert to m/s
          car_yaw = deg2rad(car_yaw);


          ego.pushState({t0, car_x, car_y, car_speed*cos(car_yaw), car_speed*sin(car_yaw)});

          // if (first_call) {
            vector<double> state0 = {t0, ego.state[1], 6, 20, 0, 0, 0};
            target.set_state(state0);
          // }

          auto state1 = target.traj.evaluate_state(20.0);
          // state1[0] += t0;
          Vehicle target2 = target;
          target2.set_state(state1);
          for (auto i : state1) {cout << i << ", ";}
          cout << endl;

          /* --------------------------------------------------------------- */
          /* Update other car states */
          /* --------------------------------------------------------------- */


        	auto sensor_fusion = j[1]["sensor_fusion"];

          // // iterate over each car and update the map of all cars
          // for (auto& car : sensor_fusion) {
          //   vector<double> car_state ({t0, car[1], car[2], car[3], car[4]});
          //   if (cars.count(car[0]) == 0)
          //     cars[car[0]] = Vehicle(road,car[0],car_state); // create the car
          //   else
          //     cars[car[0]].pushState(car_state); // the car already has been added
          // }

          /* --------------------------------------------------------------- */
          /* Get start point and end point for trajectory generation */
          /* --------------------------------------------------------------- */


        	// Previous path data given to the Planner
        	vector<double> previous_path_x = j[1]["previous_path_x"];
        	vector<double> previous_path_y = j[1]["previous_path_y"];
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

          Vehicle ego2 = ego;

          // if (previous_path_x.size() == 0) {
          //   // end_path_s = ego.state[1];
          //   // end_path_d = ego.state[2];
          //
          //   // start_state = current_state
          // } else {
          //   // start_state = end_state
          //   ego2.set_state(end_state);
          // }

          // Update ego's neighborhood
          ego.update_neighborhood(cars);

          vector<Vehicle> neighborhood;
          for (auto i : ego.neighborhood) {
            neighborhood.push_back(cars[i]);
          }

          // generate fictional target vehicle
          // Vehicle target = ego2;
          // auto target_state = target.state;
          // target_state[2] = 6;
          // target_state[3] = 5;
          // target_state[4] = 0;
          // target_state[5] = 0;
          // target_state[6] = 0;
          // target_state[1] = target_state[1] + target_state[3]*10;
          // target.set_state(target_state);

          // cout << "here0" << endl;

          cout << "current_sd :";
          for (auto i : ego2.state) cout << " " << i;
          cout << endl;
          cout << "goal_sd :";
          for (auto i : target2.state) cout << " " << i;
          cout << endl;

          cout << "current_xy :";
          for (auto i : ego2.state_xy) cout << " " << i;
          cout << endl;
          cout << "goal_xy :";
          for (auto i : target2.state_xy) cout << " " << i;
          cout << endl;

          // auto trajectories = ptg.generate(ego, target, 2.0, neighborhood);
          Trajectory traj;
          if (first_call) {
            traj = ptg.generate(ego2, target2, 2, neighborhood);
            first_call = false;
          } else {
            traj = ptg.generate(ego2, target2, 2, neighborhood);
          }


          auto trajT = traj.evaluate_state(0);
          cout << "trajectory start xy :";
          for (auto i : trajT) cout << " " << i;
          cout << endl;

          trajT = traj.evaluate_state(traj.T);
          cout << "trajectory end xy :";
          for (auto i : trajT) cout << " " << i;
          cout << endl << endl;



          // cout << "here1" << endl;
          // auto traj = trajectories.back();

          // // if (previous_path_x.size() > 1)
          // //   ego.state[1] = end_path_s;
          // ego.state[2] = 6;
          // ego.state[3] = 20;
          // ego.state[4] = 0;
          // ego.state[5] = 0;
          // ego.state[6] = 0;
          // ego.state_xy = road.Frenet_to_Cartesian(ego.state);
          // ego.predict_trajectory();




          // cout << "x traj :" << endl;
          // for (auto& i: ego.traj.x_coeffs) cout << i << ", ";
          // cout << endl;
          // cout << "y traj :" << endl;
          // for (auto& i: ego.traj.y_coeffs) cout << i << ", ";
          // cout << endl;
          // cout << "T : " << ego.traj.T << endl;

          // cout << "here2" << endl;

          plotter.update(ego,cars,road); // update plot

        	json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // if (previous_path_x.size() > 1) {
          //   for (int i = 0; i<previous_path_x.size(); ++i) {
          //     next_x_vals.push_back(previous_path_x[i]);
          //     next_y_vals.push_back(previous_path_y[i]);
          //   }
          // }

          auto traj_state = traj.generate_state(0.02);
          // cout << traj_state.size() << endl;
          for (auto& s_i : traj_state) {
            auto s_ic = road.Frenet_to_Cartesian(s_i);
            next_x_vals.push_back(s_ic[1]);
            next_y_vals.push_back(s_ic[2]);
            // cout << s_i[1] << ", " << s_i[2] <<", "<< s_ic[1] << ", " << s_ic[2] << endl;
            if (next_x_vals.size() >= 100) {
              end_state = s_i;
              break;
            }
          }

          // cout << "end state :";
          // for (auto i : end_state) cout << " " << i;
          // cout << endl;

          // cout << endl;
          // cout << previous_path_x.size() << endl;

        	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;

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
