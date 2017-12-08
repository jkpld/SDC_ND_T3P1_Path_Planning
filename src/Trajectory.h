#ifndef Trajectory_H
#define Trajectory_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace Eigen;
using namespace std;

class Trajectory {
private:

  /* diff - Compute the derivate of a polynomial */
  vector<double> diff(vector<double>& coeff) {
    vector<double> coeff_d;
    for (int i=1; i<coeff.size(); ++i) {
      coeff_d.push_back(coeff[i]*i);
    }
    return coeff_d;
  }

  /* poly_eval - Evaluate a 1D polynomial */
  double poly_eval(vector<double>& coeffs, double t) {
    double val = 0;
    for (int i=0; i<coeffs.size(); ++i) {
      val += coeffs[i]*pow(t,i);
    }
    return val;
  }

public:
  vector<double> x_coeffs; // polynomial coefficients for the x polynomial
  vector<double> y_coeffs; // polynomial coefficients for the y polynomial
  double T; // time horizon of Trajectory

  double cost; // Cost of trajectory
  bool valid; // is trajectory valid

  Trajectory() {};
  Trajectory(vector<double> xc, vector<double> yc, double t) : x_coeffs(xc), y_coeffs(yc), T(t) {};
  virtual ~Trajectory() {};

  /* differentiate - Compute and return trajectory's derivative */
  Trajectory differentiate(int n=1) {
    Trajectory traj_d = *this;

    auto tmp_xc = x_coeffs;
    auto tmp_yc = y_coeffs;
    for (int i=0; i<n; ++i) {
      tmp_xc = diff(tmp_xc);
      tmp_yc = diff(tmp_yc);
    }

    traj_d.x_coeffs = tmp_xc;
    traj_d.y_coeffs = tmp_yc;
    return traj_d;
  }

  /* evaluate - evaluate a trajectory at a single point
    t : point to evaluate the trajectory
    output : {x, y}
  */
  vector<double> evaluate(double t) {
    vector<double> xy(2);
    xy[0] = poly_eval(x_coeffs, t);
    xy[1] = poly_eval(y_coeffs, t);
    return xy;
  }
  vector<double> evaluate_state(double t) {
    // To get the state, we need to evaluate the trajectory and its first and
    // second derivatives at the time t.
    auto traj_d = differentiate();
    auto traj_dd = traj_d.differentiate();

    vector<double> state;
    auto xy = evaluate(t);
    auto vxy = traj_d.evaluate(t);
    auto axy = traj_dd.evaluate(t);

    state.push_back(t);
    state.push_back(xy[0]);
    state.push_back(xy[1]);
    state.push_back(vxy[0]);
    state.push_back(vxy[1]);
    state.push_back(axy[0]);
    state.push_back(axy[1]);

    return state;
  }

  /* generate - Generate a full trajectory from t=0 to t=T in increments of dt
    dt : increment at which to generate the trajectory
    output : { {x}, {y} }, where {x},{y} are the set of x,y points
  */
  vector<vector<double>> generate(double dt) {
    vector<double> x, y;
    vector<vector<double>> xy;

    for (double t = 0; t <= T; t += dt) {
      auto xy_i = evaluate(t);
      x.push_back(xy_i[0]);
      y.push_back(xy_i[1]);
    }
    xy.push_back(x);
    xy.push_back(y);
    return xy;
  }
  MatrixXd generate_EigMat(double dt) {
    int N = ceil(T/dt);
    MatrixXd xy(N,2);

    double t = 0;
    for (int i=0; i<N; ++i, t += dt) {
      auto xy_i = evaluate(t);
      xy(i,0) = xy_i[0];
      xy(i,1) = xy_i[1];

    }
    return xy;
  }

  /* generate - Same as generate, but computes the full state
    dt : increment at which to generate the trajectory
    output : { {t}, {x}, {y}, {vx}, {vy}, {ax}, {ay} }
  */
  vector<vector<double>> generate_state(double dt) {
    vector<vector<double>> states;

    // we could just call evaluate_state(), but that will compute the derivatives
    // many times, so we will just re-write some of the code.

    auto traj_d = differentiate();
    auto traj_dd = traj_d.differentiate();

    for (double t = 0; t <= T; t += dt) {
      vector<double> state_i;
      auto xy = evaluate(t);
      auto vxy = traj_d.evaluate(t);
      auto axy = traj_dd.evaluate(t);

      state_i.push_back(t);
      state_i.push_back(xy[0]);
      state_i.push_back(xy[1]);
      state_i.push_back(vxy[0]);
      state_i.push_back(vxy[1]);
      state_i.push_back(axy[0]);
      state_i.push_back(axy[1]);

      states.push_back(state_i);
    }
    return states;
  }
  MatrixXd generate_state_EigMat(double dt) {
    int N = ceil(T/dt);
    MatrixXd state(N,7);

    auto traj_d = differentiate();
    auto traj_dd = traj_d.differentiate();

    double t = 0;
    for (int i=0; i<N; ++i, t += dt) {
      vector<double> state_i;
      auto xy = evaluate(t);
      auto vxy = traj_d.evaluate(t);
      auto axy = traj_dd.evaluate(t);

      state(i,0) = t;
      state(i,1) = xy[0];
      state(i,2) = xy[1];
      state(i,3) = vxy[0];
      state(i,4) = vxy[1];
      state(i,5) = axy[0];
      state(i,6) = axy[1];
    }
    return state;
  }
};

/* ------------------------------------------------------------------------- */
/* Function definitions */
/* ------------------------------------------------------------------------- */



















#endif
