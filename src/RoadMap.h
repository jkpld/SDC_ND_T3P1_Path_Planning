#ifndef RoadMap_H
#define RoadMap_H

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "json.hpp"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

const bool BETTER_FRENET_APPROX = true;
const double RoadLength = 6945.554;

class RoadMap {
private:
  int num_wp;
  MatrixXd wp_r;
  MatrixXd wp_n;
  VectorXd wp_s;

  /* ClosestWaypoint - Get the index of the closest waypoint to position (x,y)
    x : global x coordinate
    y : global y coordinate
  */
  int ClosestWaypoint(double x, double y);

  /* NextWaypoint - Get the index of the next waypoint infront of position (x,y)
    x : global x coordinate
    y : global y coordinate
    theta : orientation at the point (x,y) - the direction we are looking
  */
  int NextWaypoint(double x, double y, double theta);

  /* ComputeTransformMatrices - get the 2x2 transform matrix from cartesian to
    frenet and from frenet to cartesian.
    wp0_idx : index of the previous waypoint
    sp1_idx : index of the next waypoint

    output : { Matrix2d C2F, Matrix2d F2C }
  */
  vector<Matrix2d> ComputeTransformMatrices(int wp0_idx, int wp1_idx, double dist_along_path);

public:

  double lane_width = 4; // [m]

  RoadMap(string map_file);
  virtual ~RoadMap();

  void LoadWaypoints(string map_file);

  /* Cartesian_to_Frenet - Take in a vehicle state (t,x,y,vx,vy,ax,ay) and
    return the state in Frenet coordinates (t,s,d,vs,vd,as,ad)
    state : veicle state, 7 element vector (t,x,y,vx,vy,ax,zy)
  */
  vector<double> Cartesian_to_Frenet(vector<double> state);

  /* Frenet_to_Cartesian - Take in a vehicle state is frenet coordinates
    (t,s,d,vs,vd,as,ad) and return the state in Cartesian coordinates
    (t,x,y,vx,vy,ax,ay)
    state : veicle state, 7 element vector (t,s,d,vs,vd,as,ad)
  */
  vector<double> Frenet_to_Cartesian(vector<double> state);

  vector<vector<double>> get_waypoints_as_vector();
  vector<MatrixXd> get_waypoints();

};



#endif
