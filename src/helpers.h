#ifndef HELPERS_H
#define HELPERS_H

#include "Eigen-3.3/Eigen/Core"
#include "polynomial.h"

using namespace Eigen;
using namespace std;

class Trajectory {
private:
  PP pp

public:

  Trajectory(VectorXd coef1) : pp(PP(coef1)) {};
  Trajectory(VectorXd coef1, VectorXd coef2, double k) : pp(PP(coef1, coef2, k)) {};

  VectorXd state_at(double t) {
    VectorXd state = pp.ppeval(t);

    // Ensure the state has at least 3 elements.
    VectorXd state3 = VectorXd::Zero(3);
    int N = state.size();
    if (N<3) state3.head(N) = pp;
    else state3 = pp.head(3);

    return state3;
  }
};

class State {
public:
  Vector3d x
  Vector3d y

  State() : x(Vector3d::Zero()), y(Vector3d::Zero()) {};
  State(Vector3d x, Vector3d y) : x(x), y(y) {};

  vector<Trajectory> get_trajectory() {
    Vector3d xt = x;
    Vector3d yt = y;
    xt(2) *= 0.5;
    yt(2) *= 0.5;
    return {Trajectory(xt),Trajectory(yt)};
  }
};

#endif
