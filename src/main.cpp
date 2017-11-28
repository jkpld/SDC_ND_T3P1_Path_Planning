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
  plotter.CordSys = VisualDebugger::FRENET;

  map<int, Vehicle> cars;
  auto startTime = start_timer();
  Vehicle ego = Vehicle();


  h.onMessage([&road, &cars, &ego, &startTime](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];

          car_yaw = deg2rad(car_yaw);
          car_speed = car_speed*0.44704;

          // Update ego car, initialize if first iteration
          if (ego.ID == 999) {
            ego.ID = 0;
            vector<double> car_state ({stop_timer(startTime), car_x, car_y, car_speed*cos(car_yaw), car_speed*sin(car_yaw), 0, 0});
            ego.state = car_state;
          } else {
            vector<double> car_state ({stop_timer(startTime), car_x, car_y, car_speed*cos(car_yaw), car_speed*sin(car_yaw)});
            ego.pushState(car_state);
          }

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	// Previous path's end s and d values
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];

          // iterate over each car and update the map of all cars
          for (auto& car : sensor_fusion) {
            vector<double> car_state ({stop_timer(startTime), car[1], car[2], car[3], car[4]});
            if (cars.count(car[0]) > 0) { // the car already has been added
              cars[car[0]].pushState(car_state);
            }
            else {
              cars[car[0]] = Vehicle(car[0],car_state);
            }
          }

          // Update ego's neighborhood
          ego.update_neighborhood(cars);

          plotter.update(ego,cars,road);
          // update_plot(ego,cars,road);

          // for (auto i : ego.neighborhood) {
          //   cout << i << ", ";
          // }
          // cout << endl;

        	json msgJson;

        	vector<double> next_x_vals;
        	vector<double> next_y_vals;


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
