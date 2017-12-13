#ifndef PCHIP_H
#define PCHIP_H

#include <vector>
#include <cassert>
#include <algorithm>
#include <iostream>

// using namespace Eigen;
using namespace std;

class pchip {
public:
  vector<vector<double>> coeffs;

  inline double sign(double x) {return (x>0) ? 1 : ((x==0) ? 0 : -1);};

public:
  vector<double> x;
  vector<double> y;

  // Construct the pchip interpolator
  pchip() {};
  pchip(vector<double> xpts, vector<double> ypts) : x(xpts), y(ypts) {

    // Array size
    int N = x.size();

    // Ensure that x and y are the same size
    assert(N == y.size());

    // Initialize arrays
    coeffs.assign(N-1, vector<double>(4,0));
    vector<double> dx(N-1);
    vector<double> del(N-1);
    vector<double> slopes(N);

    /* construct derivatives */
    // initialize first point
    dx[0] = x[1] - x[0];
    del[0] = (y[1] - y[0]) / dx[0];

    // interior points
    for (int i=2; i<N; ++i) {
      x[i] = x[i];
      y[i] = y[i];

      dx[i-1] = x[i] - x[i-1];
      del[i-1] = (y[i] - y[i-1]) / dx[i-1];

      if (sign(del[i-2])*sign(del[i-1]) > 0) {
        double h = dx[i-2];
        double hs = dx[i-2] + dx[i-1];
        double w1 = (dx[i-2] + hs)/(3*hs);
        double w2 = (hs + dx[i-1])/(3*hs);

        double dmax = fmax(fabs(del[i-2]), fabs(del[i-1]));
        double dmin = fmin(fabs(del[i-2]), fabs(del[i-1]));

        slopes[i-1] = dmin / (w1*del[i-2]/dmax + w2*del[i-1]/dmax );
      }
    }

    // end points
    slopes[0] = ( (2*dx[0] + dx[1])*del[0] - dx[0]*del[1] ) / (dx[0] + dx[1]);
    if (sign(slopes[0]) != sign(del[0])) {
      slopes[0] = 0;
    } else if (sign(del[0]) != sign(del[1]) && (fabs(slopes[0]) > fabs(3*del[0]))) {
      slopes[0] = 3*del[0];
    }

    slopes[N-1] = ( (2*dx[N-2] + dx[N-3])*del[N-2] - dx[N-2]*del[N-3])/(dx[N-2] + dx[N-3]);
    if (sign(slopes[N-1]) != sign(del[N-2])) {
      slopes[N-1] = 0;
    } else if (sign(del[N-2]) != sign(del[N-3]) && (fabs(slopes[N-1]) > fabs(3*del[N-2]))) {
      slopes[N-1] = 3*del[N-2];
    }

    for (int i=0; i<N-1; ++i) {
      // construct polynomial coefficients
      double dzzdx = (del[i] - slopes[i]) / dx[i];
      double dzdxdx = (slopes[i+1] - del[i]) / dx[i];

      coeffs[i][3] = (dzdxdx - dzzdx) / dx[i];
      coeffs[i][2] = 2*dzzdx - dzdxdx;
      coeffs[i][1] = slopes[i];
      coeffs[i][0] = y[i];
    }

  };
  virtual ~pchip() {};

  vector<vector<double>> evaluate(vector<double>& x_ii) {
    vector<vector<double>> y_ii;

    for (auto x_i : x_ii) {
      y_ii.push_back(evaluate(x_i));
    }

    return y_ii;
  };

  vector<double> evaluate(const double& x_ii, int *idx = 0) {
    /*
      Return a vector giving {y, y', y'', y'''} evaluated at x_ii

      If the optional parameter idx is given, then we will use that for the
      index of the pchip section. If the value of idx is less than zero, then
      we will search for what index x_ii refers to, and then return that value
      using the *idx.

      This will allow for multiple evaluations using the same x_ii value, but
      only searching for the proper index once.
      
      Ex.
      pchip ppx = pchip(t,x);
      pchip ppy = pchip(t,y);

      int idx = -1;

      auto x_i = ppx.evaluate(t_i, &idx); // computes the index
      auto y_i = ppy.evaluate(t_i, &idx); // uses the index already computed
    */

    int ii;

    // if a index value was given, then extract the value
    if (idx) {
      ii = *idx;
    } else {
      ii = -1;
    }

    // if the index value given is less than 0, then compute the proper index
    if (ii < 0) {
      vector<double>::const_iterator it;
      it = lower_bound(x.begin(),x.end(),x_ii);
      ii = max( int(it-x.begin())-1, 0);

      // if an index value was given, overwrite it with the proper index
      if (idx) {
        *idx = ii;
      }
    }

    double dx = x_ii - x[ii];

    double y_ii = ((coeffs[ii][3]*dx + coeffs[ii][2])*dx + coeffs[ii][1])*dx + coeffs[ii][0];
    double yd_ii = (coeffs[ii][3]*dx*3 + coeffs[ii][2]*2)*dx + coeffs[ii][1];
    double ydd_ii = coeffs[ii][3]*dx*6 + coeffs[ii][2]*2;
    double yddd_ii = coeffs[ii][3]*6;

    return {y_ii, yd_ii, ydd_ii, yddd_ii};
  };
};

#endif
