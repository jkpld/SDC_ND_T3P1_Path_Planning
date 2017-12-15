#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "polynomial.h"


/* Contents

class Trajectory - Wrapper of PP class (polynomial.h) that includes method
  state_at(t) that returns a Vector3d giving the (position, speed, acceleration)
  at time t.

class State - Hold the state (position, speed, acceleration) for both
  dimensions, and has a method get_trajectory() for returning a vector with two
  Trajectory's, one for each dimension.

enum class Modes - types of active modes

struct ActiveMode - Hold active mode data as described in [1].

struct SearchMode - Holds a vector of ActiveMode's along with a goal_lane.

void ind2sub(N,idx,i,j) - Convert from linear index (idx) to sub-index (i,j)
  given the number of rows in a matrix (N).

vector sort_outer_sum(a,b) - Return the indices of the sorted matrix
  C[i,j] = a[i]+b[j]

class Rectangle - Class for a rotated rectangle with a method overlap() that
  determines if two Rectangles overlap.

*/


using namespace Eigen;
// using namespace std;
using std::vector;
using std::cout;
using std::endl;

class Trajectory : public PP{
public:

  Trajectory() : PP() {};
  Trajectory(VectorXd coef1) : PP(coef1) {};
  Trajectory(VectorXd coef1, VectorXd coef2, double k) : PP(coef1, coef2, k) {};

  Vector3d state_at(double t) const {
    VectorXd s = ppeval(t);

    // Ensure the state has at least 3 elements.
    Vector3d state = Vector3d::Zero();
    int N = s.size();
    if (N<3) state.head(N) = s;
    else state = s.head(3);

    return state;
  }

  Trajectory trajectory_at(double t) const {
    // Get the derivatives of the polynomial at t.
    VectorXd coef_d = ppeval(t);

    // Convert the derivatives to the coefficients by integrating - dividing by
    // (1:N)!. See note in polynomial/polyeval().
    int N = coef_d.size();
    VectorXd fac = VectorXd::LinSpaced(N,0,N-1);
    fac(0) = 1;
    for (int i=1; i<N; ++i) fac(i) *= fac(i-1);
    VectorXd coef_n = coef_d.cwiseQuotient(fac);

    if (coef.size() == 1 || t>=knot) {
      return Trajectory(coef_n);
    } else {
      return Trajectory(coef_n, coef[1], knot-t);
    }
  }
};

class State {
public:
  Vector3d x;
  Vector3d y;

  State() : x(Vector3d::Zero()), y(Vector3d::Zero()) {};
  State(Vector3d x, Vector3d y) : x(x), y(y) {};

  vector<Trajectory> get_trajectory() const {
    Vector3d xt = x;
    Vector3d yt = y;
    xt(2) *= 0.5;
    yt(2) *= 0.5;
    return {Trajectory(xt),Trajectory(yt)};
  }

  void display() const {
    cout << "  x : " << x.transpose() << endl;
    cout << "  y : " << y.transpose() << endl;
  }
};

enum class Mode : char {FOLLOWING, MERGING, STOPPING, VELOCITY_KEEPING, LATERAL};

struct ActiveMode {
  Mode mode;
  vector<State> state;
  double number;

  ActiveMode(Mode mode, State state) : mode(mode), state({state}), number(0) {};
  ActiveMode(Mode mode, vector<State> state) : mode(mode), state(state), number(0) {};
  ActiveMode(Mode mode, double number) : mode(mode), number(number), state({State()}) {};


};


struct SearchMode {
  vector<ActiveMode> activeModes;
  double goal_lane;
  double min_lv_speed = -1;


  SearchMode(ActiveMode aM, double gl) : SearchMode(vector<ActiveMode>(1,aM), gl, -1.0) {};
  SearchMode(vector<ActiveMode> aMs, double gl) : SearchMode(aMs, gl, -1.0) {};
  SearchMode(ActiveMode aM, double gl, double mlv_s) : SearchMode(vector<ActiveMode>(1,aM), gl, mlv_s) {};
  SearchMode(vector<ActiveMode> aMs, double gl, double mlv_s) : activeModes(aMs), goal_lane(gl), min_lv_speed(mlv_s) {};
};

void ind2sub(int const& size1, int const& idx, int& v1, int& v2) {
  v1 = idx % size1;
  v2 = (idx - v1)/size1;
}


vector<size_t> sort_outer_sum(const vector<double> &v, const vector<double> &v2) {
  // initialize original linear index locations
  auto N = v.size();
  vector<size_t> idx(N*v2.size());
  iota(idx.begin(), idx.end(), 0);

  // sort linear indexes based on comparing values the matrix v[i] + v[j]
  std::sort(idx.begin(), idx.end(),
       [&v,&v2,&N](size_t i1, size_t i2) {
         int m1,n1,m2,n2;
         ind2sub(N,i1,m1,n1);
         ind2sub(N,i2,m2,n2);
         return (v[m1]+v2[n1]) < (v[m2]+v2[n2]);
       });

  return idx;
}

class Rectangle {
public:

  Rectangle() : Rectangle(1,1,0,0,0) {}
  Rectangle(double L, double W) : Rectangle(L,W,0,0,0) {}
  Rectangle(double L, double W, double th) : Rectangle(L,W,th,0,0) {}
  Rectangle(double L, double W, double th, double xc, double yc) : Length(L), Width(W), x_center(xc), y_center(yc), orientation(th), bbox_verts((MatrixXd(2,4)<<-1,-1,1,1,-1,1,1,-1).finished()) {set_bbox_size();};

  void set_size(double L, double W) {
    Length = L;
    Width = L;
    set_bbox_size();
  }

  void set_pose(double th, double xc, double yc) {
    orientation = th;
    x_center = xc;
    y_center = yc;
    set_bbox_rotation();
  }

  void set_orientation(double th) {
    orientation = th;
    set_bbox_rotation();
  }

  void set_center(double xc, double yc) {
    x_center = xc;
    y_center = yc;
    set_bbox_center();
  }

  MatrixXd operator()() {return bbox;}

  bool overlap(Rectangle& rec2, bool exact = true) const {
    // Determine if *this rectangle overlaps with rectangle rec2
    // If exact is false, then only the axis aligned bounding boxes will be
    // used to determine if they overlap. If exact is true, then we check to see
    // if the rotated rectangles overlap or not.

    MatrixXd bbox2 = rec2();

    // First check if the axis aligned bboxes overlap, and if they do, then
    // check to see if the rectangles actually overlap.
    Array2d v1max = bbox.rowwise().maxCoeff();
    Array2d v2min = bbox2.rowwise().minCoeff();

    if ((v2min > v1max).any()) {
      return false;
    } else {
      Array2d v2max = bbox2.rowwise().maxCoeff();
      Array2d v1min = bbox.rowwise().minCoeff();
      if ((v1min > v2max).any()) {
        return false;
      } else {
        if (exact) {
          /* The axis aligned boxes overlap, so now check more precisly if the
          rotated boxes overlap */

          // Get the two unique normal vectors of each rectangle
          MatrixXd n(2,4);
          n << bbox.block<2,2>(0,1) - bbox.block<2,2>(0,0), bbox2.block<2,2>(0,1) - bbox2.block<2,2>(0,0);
          n.transposeInPlace(); // 4x2, each row is a unit vector

          Matrix4d p1 = n*bbox; // 4x4, each each row is the projection of the 4 vertices along a specific normal
          Matrix4d p2 = n*bbox2; // 4x4, each each row is the projection of the 4 vertices along a specific normal

          // Check if the minimum of one shape is larger than the maximum of another
          Array4d p1min = p1.rowwise().minCoeff();
          Array4d p2max = p2.rowwise().maxCoeff();

          if ((p1min > p2max).any()) {
            return false;
          } else {
            Array4d p1max = p1.rowwise().maxCoeff();
            Array4d p2min = p2.rowwise().minCoeff();
            return !((p2min > p1max).any());
          }
        } else {
          return true;
        }
      }
    }
  }

private:
  double Length;
  double Width;
  double x_center;
  double y_center;
  double orientation;

  MatrixXd bbox_verts;
  MatrixXd bbox_centered;
  MatrixXd bbox_rotated;
  MatrixXd bbox;

  void set_bbox_size() {
    Array2d lw;
    lw << Length, Width;

    bbox_centered = (bbox_verts.array().colwise() * lw * 0.5).matrix();
    set_bbox_rotation();
    set_bbox_center();
  }

  void set_bbox_rotation() {
    Matrix2d rot;
    double c = cos(orientation);
    double s = sin(orientation);
    rot << c, -s, s, c;

    bbox_rotated = rot*bbox_centered;
    set_bbox_center();
  }

  void set_bbox_center() {
    Vector2d c;
    c << x_center, y_center;
    bbox = bbox_rotated.colwise()  + c;
  }
};

#endif
