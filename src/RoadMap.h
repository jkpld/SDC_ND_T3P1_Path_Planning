#ifndef RoadMap_H
#define RoadMap_H

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <functional>

#include "pchip.h"
// #include "polynomial.h"
#include "spline.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "json.hpp"

// using namespace std;
using namespace Eigen;

using std::vector;
using std::string;
using json = nlohmann::json;

class RoadMap {
private:
  int num_wp;
  MatrixXd wp_r;
  MatrixXd wp_n;
  VectorXd wp_s;

  // pchip x_road;
  // pchip y_road;


  /* ClosestWaypoint - Get the index of the closest waypoint to position (x,y)
    x : global x coordinate
    y : global y coordinate
  */
  // int ClosestWaypoint(double x, double y) {
  //   // Initialize closestWaypoint index
  //   MatrixXd::Index closestWaypoint;
  //
  //   // Create vector with current point
  //   Vector2d r;
  //   r << x, y;
  //
  //   // Compute waypoint that is closest to the current point
  //   (wp_r.rowwise() - r.transpose()).rowwise().squaredNorm().minCoeff(&closestWaypoint);
  //
  //   // cout << "Closest Waypoint : " << wp_r(closestWaypoint,0) << ", " << wp_r(closestWaypoint,1) << " : idx = " << closestWaypoint << endl;
  //
  // 	return closestWaypoint;
  // }

  /* NextWaypoint - Get the index of the next waypoint infront of position (x,y)
    x : global x coordinate
    y : global y coordinate
    theta : orientation at the point (x,y) - the direction we are looking
  */
  // int NextWaypoint(double x, double y, double theta) {
  //
  // 	int closestWaypoint = ClosestWaypoint(x,y);
  //
  //   Vector2d wp0 = wp_r.row(closestWaypoint);
  //   Vector2d wp0n = wp_n.row(closestWaypoint);
  //   Vector2d wp0t;
  //   Vector2d r;
  //   wp0t << -wp0n(1), wp0n(0);
  //   r << x, y;
  //
  //   if ((r-wp0).dot(wp0t) > 0)
  //   {
  //     closestWaypoint++;
  //     if (closestWaypoint == num_wp)
  //     {
  //       closestWaypoint = 0;
  //     }
  //   }
  //
  //   // auto m_x = wp_r(closestWaypoint,0);
  //   // auto m_y = wp_r(closestWaypoint,1);
  //   //
  // 	// double heading = atan2((m_y-y),(m_x-x));
  //   //
  // 	// double angle = fabs(theta-heading);
  //   // angle = min(2*M_PI - angle, angle);
  //   //
  //   // if(angle > M_PI/4)
  //   // {
  //   //   closestWaypoint++;
  //   //   if (closestWaypoint == num_wp)
  //   //   {
  //   //     closestWaypoint = 0;
  //   //   }
  //   // }
  //
  //   // cout << "Nearest Waypoint : " << wp_r(closestWaypoint,0) << ", " << wp_r(closestWaypoint,1) << " : idx = " << closestWaypoint << endl << endl;
  //
  //   return closestWaypoint;
  // }

  /* ComputeTransformMatrices - get the 2x2 transform matrix from cartesian to
    frenet and from frenet to cartesian.
    wp0_idx : index of the previous waypoint
    sp1_idx : index of the next waypoint

    output : { Matrix2d C2F, Matrix2d F2C }
  */
  // vector<Matrix2d> ComputeTransformMatrices(int wp0i, int wp1i, VectorXd dist_along_path) {
  //   Vector2d wp0 = wp_r.row(wp0i);
  //   Vector2d wp1 = wp_r.row(wp1i);
  //   Vector2d wp0n = wp_n.row(wp0i);
  //   Vector2d wp1n = wp_n.row(wp1i);
  //
  //   Vector2d s_uv = wp1 - wp0; // vector between waypoints
  //   double d = s_uv.norm(); // distance between waypoints
  //   s_uv /= d; // unit vector between the two waypoints
  //
  //   // just make d_uv perpendicular to w (rotated 90o clockwise)
  //   auto d_uv = s_uv;
  //   d_uv << s_uv(1), -s_uv(0);
  //
  //   Matrix2d C2F, F2C;
  //   C2F << s_uv.transpose(), d_uv.transpose();
  //   F2C  = C2F.inverse();
  //
  //   return {C2F, F2C};
  // }

public:
  PP x_pp;
  PP y_pp;

  double RoadLength;
  double lane_width; // [m]
  std::function<double(double)> lonDistCorrection;

  RoadMap(string map_file) : RoadLength(6945.554),
                             lonDistCorrection([=](double ds) ->double {
                                    while (ds < -RoadLength/2) ds += RoadLength;
                                    while (ds > RoadLength/2) ds -= RoadLength;
                                    return ds;
                                  }),
                             lane_width(4)
  {
    LoadWaypoints(map_file);
  }

  void LoadWaypoints(string map_file) {

    // temporary vectors
    vector<double> mx,my,ms,mdx,mdy;

    // Waypoint map to read from
    // string map_file_ = "../data/highway_map.csv";

    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);
    // std::ofstream out_map_("somefile.txt");

    string line;
    while (getline(in_map_, line)) {
    	std::istringstream iss(line);
    	double x,y;
    	float s, dx, dy;
    	iss >> x;
    	iss >> y;
    	iss >> s;
    	iss >> dx;
    	iss >> dy;
    	mx.push_back(x);
    	my.push_back(y);
    	ms.push_back(s);
      mdx.push_back(dx);
      mdy.push_back(dy);
    }

    // Number of waypoints
    num_wp = mx.size();

    // Initialize Matrix size
    wp_r = MatrixXd::Zero(num_wp,2);
    wp_n = MatrixXd::Zero(num_wp,2);
    wp_s = VectorXd::Zero(num_wp);

    for (int i=0; i<num_wp; i++) {
      wp_r.row(i) << mx[i], my[i];
      wp_n.row(i) << mdx[i], mdy[i];
      wp_s(i) = ms[i];
    }

    // Make vectors cyclic
    mx.insert(mx.begin(), mx.back());
    my.insert(my.begin(), my.back());
    ms.insert(ms.begin(), ms.back()-RoadLength);

    mx.push_back(mx[1]);
    my.push_back(my[1]);
    ms.push_back(RoadLength);
    mx.push_back(mx[2]);
    my.push_back(my[2]);
    ms.push_back(RoadLength+ms[2]);

    // for(size_t i =0; i<ms.size(); ++i) {
    //     out_map_ << mx[i] << "\t" << my[i] << "\t" << ms[i] << '\n';
    // }

    // Create two peicewise continuous hermite spline interplantes using s for the
    // independent variable and x/y as the dependent varaibles
    // x_road = pchip(ms,mx);
    // y_road = pchip(ms,my);

    tk::spline x_spline, y_spline;
    x_spline.set_points(ms,mx);
    y_spline.set_points(ms,my);
    //
    // write the splines to a PP class object.
    vector<VectorXd> x_coef(x_spline.m_a.size(), VectorXd::Zero(4));
    vector<double> x_knot(x_spline.m_a.size());
    vector<VectorXd> y_coef(x_spline.m_a.size(), VectorXd::Zero(4));
    vector<double> y_knot(x_spline.m_a.size());
    for (int i = 0; i<x_spline.m_a.size(); ++i) {
      x_coef[i] << x_spline.m_y[i], x_spline.m_c[i], x_spline.m_b[i], x_spline.m_a[i];
      x_knot[i] = x_spline.m_x[i];
      y_coef[i] << y_spline.m_y[i], y_spline.m_c[i], y_spline.m_b[i], y_spline.m_a[i];
      y_knot[i] = x_spline.m_x[i];
    }
    x_pp = PP(x_coef, x_knot);
    y_pp = PP(y_coef, y_knot);
    //
    //
    //
    // for(double i =0; i<6946.1; i+=2) {
    //   VectorXd xds = x_pp.ppeval(i);
    //   VectorXd yds = y_pp.ppeval(i);
    //
    //     out_map_ << xds[0] << ", " << xds[1] << ", " << xds[2]<< ", " << xds[3]<< ", " << yds[0]<< ", " << yds[1]<< ", " << yds[2]<< ", " << yds[3] << '\n';
    //
    //
    // }
  }

  double curvature(double s) const {
    s = fmod(s, RoadLength);
    VectorXd rx = x_pp.ppeval(s);
    VectorXd ry = y_pp.ppeval(s);
    return (rx(2)*ry(1) - rx(1)*ry(2))/pow(rx(1)*rx(1) + ry(1)*ry(1),1.5);
  }

  Matrix2d TransformMat_C2F(double s) {
    s = fmod(s, RoadLength);

    // get x, x', y, y' of the road center line for the current s location

    VectorXd x_dat = x_pp.ppeval(s);
    VectorXd y_dat = y_pp.ppeval(s);

    // road tangent vector
    Vector2d t;
    t << x_dat[1], y_dat[1];
    t /= t.norm();

    // road normal vector
    Vector2d n;
    n << t(1), -t(0);

    Matrix2d C2F;
    C2F << t.transpose(), n.transpose();

    return C2F;
  }
  Matrix2d TransformMat_F2C(double s) {
    Matrix2d C2F = TransformMat_C2F(s);
    Matrix2d F2C = C2F.inverse();
    return F2C;
  }
  vector<double> getXY(double s, double d) {
    s = fmod(s, RoadLength);

    VectorXd x_dat = x_pp.ppeval(s);
    VectorXd y_dat = y_pp.ppeval(s);

    Vector2d r;
    r << x_dat[0], y_dat[0];

    // road tangent vector
    Vector2d t;
    t << x_dat[1], y_dat[1];
    t /= t.norm();

    // road normal vector
    Vector2d n;
    n << t(1), -t(0);

    r += d*n;

    return {r(0), r(1)};
  }


  /* Cartesian_to_Frenet - Take in a vehicle state (t,x,y,vx,vy,ax,ay) and
    return the state in Frenet coordinates (t,s,d,vs,vd,as,ad)
    state : veicle state, 7 element vector (t,x,y,vx,vy,ax,zy)
  */
  // vector<double> Cartesian_to_Frenet(vector<double> state) {
  //
  //   // indices of last and next waypoints
  //   double theta = atan2(state[4],state[3]);
  // 	int wp1i = NextWaypoint(state[1], state[2], theta);
  //   int wp0i = (wp1i == 0) ? num_wp - 1 : wp1i - 1;
  //
  //   // cartesian vectors
  //   Vector2d r, v, a, r0;
  //   r << state[1], state[2]; // location, cartesian
  //   v << state[3], state[4]; // velocity, cartesian
  //   a << state[5], state[6]; // acceleration, cartesian
  //   r0 = wp_r.row(wp0i);
  //   r = r - r0; // location relative to wp0, cartesian
  //
  //   VectorXd dist_along_path(2);
  //   // dist_along_path << u(0), u(1);
  //   // double dist_along_path = 0;
  //   auto transMat = ComputeTransformMatrices(wp0i, wp1i, dist_along_path);
  //   auto C2F = transMat[0];
  //
  //   auto rf = C2F*r;
  //   auto vf = C2F*v;
  //   auto af = C2F*a;
  //
  //
  //   vector<double> fstate (state);
  //   fstate[1] = rf[0] + wp_s(wp0i);
  //   fstate[2] = rf[1];
  //   fstate[3] = vf[0];
  //   fstate[4] = vf[1];
  //   fstate[5] = af[0];
  //   fstate[6] = af[1];
  //
  //   return fstate;
  //
  // }
  //
  // /* Frenet_to_Cartesian - Take in a vehicle state is frenet coordinates
  //   (t,s,d,vs,vd,as,ad) and return the state in Cartesian coordinates
  //   (t,x,y,vx,vy,ax,ay)
  //   state : veicle state, 7 element vector (t,s,d,vs,vd,as,ad)
  // */
  // vector<double> Frenet_to_Cartesian(vector<double> fstate) {
  //
  //   // get x, x', y, y' of the road center line for the current s location
  //   int idx = -1;
  //   vector<double> x_dat = x_road.evaluate(fstate[1], &idx);
  //   vector<double> y_dat = y_road.evaluate(fstate[1], &idx);
  //
  //   // x-y location of road center line
  //   Vector2d r;
  //   r << x_dat[0], y_dat[0];
  //
  //   // road tangent vector
  //   Vector2d t;
  //   t << x_dat[1], y_dat[1];
  //   t /= t.norm();
  //
  //   // road normal vector
  //   Vector2d n;
  //   n << t(1), -t(0);
  //
  //   // x-y location of car
  //   r += fstate[2]*n;
  //
  //   // frenet vectors
  //   Vector2d vf, af;
  //   vf << fstate[3], fstate[4]; // velocity, frenet
  //   af << fstate[5], fstate[6]; // acceleration, frenet
  //
  //   Matrix2d C2F, F2C;
  //   C2F << t.transpose(), n.transpose();
  //   F2C  = C2F.inverse();
  //
  //   auto v = F2C*vf;
  //   auto a = F2C*af;
  //
  //   vector<double> state (fstate);
  //   state[1] = r[0];
  //   state[2] = r[1];
  //   state[3] = v[0];
  //   state[4] = v[1];
  //   state[5] = a[0];
  //   state[6] = a[1];
  //
  // 	return state;
  // }
  //
  // vector<vector<double>> get_waypoints_as_vector() {
  //   vector<vector<double>> wp;
  //   vector<double> x, y, s, nx, ny;
  //
  //   for (int i=0; i<num_wp; i++) {
  //     x.push_back(wp_r(i,0));
  //     y.push_back(wp_r(i,1));
  //     s.push_back(wp_s(i));
  //     nx.push_back(wp_n(i,0));
  //     ny.push_back(wp_n(i,1));
  //   }
  //
  //   wp.push_back(x);
  //   wp.push_back(y);
  //   wp.push_back(s);
  //   wp.push_back(nx);
  //   wp.push_back(ny);
  //
  //   return wp;
  //
  // }
  // vector<MatrixXd> get_waypoints() {
  //   vector<MatrixXd> wp;
  //   wp.push_back(wp_r);
  //   wp.push_back(wp_n);
  //   return wp;
  // }

};



#endif
