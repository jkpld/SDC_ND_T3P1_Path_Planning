#ifndef POLYNOMIAL_FUNCTIONS_H
#define POLYNOMIAL_FUNCTIONS_H

/*
Polynomial definision : vector of coeficients where the index is the same as
the power. Example :

a = (a0, a1, a2, a3, ..., an)
A(x) = a(i)*x^i
*/

#include "Eigen-3.3/Eigen/Core"
#include <math.h>
#include <iostream>

using namespace Eigen;
using namespace std;

/* polyder : compute the derivative of a polynomial. */
VectorXd polyder(VectorXd const& coef) {
  int N = coef.size();
  return coef.tail(N-1).cwiseProduct(VectorXd::LinSpaced(N-1,1,N-1));
};
/* // If you want the vector to remain the same length use this
VectorXd coefd = VectorXd::Zero(N);
coefd.head(N-1) = coef.tail(N-1).cwiseProduct(VectorXd::LinSpaced(N-1,1,N-1));
return coefd;
*/

/* polyint : integrate a polynomial with constant of integration C. */
VectorXd polyint(VectorXd const& coef, double C = 0) {
  int N = coef.size();
  VectorXd coefi = VectorXd::Zero(N+1);
  coefi.tail(N) = coef.cwiseQuotient(VectorXd::LinSpaced(N,1,N));
  coefi(0) = C;
  return coefi;
};

/* polyval : evaluate a polynomial at xi */
double polyval(VectorXd const& coef, double xi) {
  int N = coef.size();
  double yi = coef(N-1);

  for (int i=N-2; i>=0; --i) {
    yi = (yi*xi + coef(i));
  }
  return yi;
};

/* polyeval : compute all derivatives of a polynomial at xi (including the 0th
derivative, which is just normal evaluation.) */
VectorXd polyeval(VectorXd const& coef, double xi) {
/* POLYEVAL Evaluate all derivatives of a polynomial at the point x0 and
also return the coefficients of the polynomial with origin translated to
x0.

A_d = polyeval(A, x0)

Input :
A - 1 x Na array giving the coefficients of a polynomial such that
  A(x) = A(1) + A(2)*x + A(3)*x^2 ... = A(i+1)*x^(i).
  Note this is the opposite ordering as Matlabs polynomials that use
  polyval.

Output :
A_d - numel(x0) x Na matrix where the i,j'th entry gives the
  (j-1)'th derivative evaluated at x0(i).

Note. The coefficients of the polynomial relative to x0, that is
  A_x0(i,j) x^j = A_j (x + x0(i))^j
are simply that A_x0(i) = A_d(i) / i!;

If you only need the value of the polynomial at x0, then it is faster to
just use polyval(A,x0).

Theory:

A(x) = a_j x^j = b_j (x-x0)^j
A(x+x0) = a_j (x+x0)^j = b_j x^j

(The above are using implicit summation over j.) Now solve for b_j using
binomial theorem. The derivatives of A at x0 are then simply

d^nA/dx^n (x0) = b_j * j!

James Kapaldo
*/
  int N = coef.size();

  // Compute a factorial array and the powers of xi
  VectorXd fac = VectorXd::LinSpaced(N,0,N-1);
  VectorXd xip = VectorXd::Ones(N);
  fac(0) = 1;
  if (N>1) {
    xip(1) = xi;
    for (int i=2; i<N; ++i) {
      fac(i) *= fac(i-1);
      xip(i) = xip(i-1)*xi;
    }
  }

  xip = xip.cwiseQuotient(fac); // xi^j/j!
  VectorXd f = coef.cwiseProduct(fac); // a(j) * j!

  VectorXd Ad = VectorXd::Zero(N);
  // The n'th derivative at xi is a convolution between f and xip
  for (int r=0; r<N; ++r) {
    Ad(r) = f.tail(N-r).dot(xip.head(N-r));
    // for (int j=r; j<N; ++j) {
    //   Ad(r) += f(j)*xip(j-r);
    // }
  }
  return Ad;
};

/* PP : Very simple piece-wise polynomial class for only 2 pieces. */
class PP {
public:
  vector<VectorXd> coef;
  double knot;

  PP() : coef({VectorXd::Zero(3)}), knot(1e300) {}
  PP(VectorXd coef1) : coef({coef1}), knot(1e300) {}
  PP(VectorXd coef1, VectorXd coef2, double k) : coef({coef1, coef2}), knot(k) {}

  double ppval(double xi) {
    if (coef.size() == 1) {
      return polyval(coef[0], xi);
    } else {
      if (xi >= knot) {
        return polyval(coef[1], xi-knot);
      } else {
        return polyval(coef[0], xi);
      }
    }
  }

  VectorXd ppeval(double xi) {
    if (coef.size() == 1) {
      return polyeval(coef[0], xi);
    } else {
      if (xi >= knot) {
        return polyeval(coef[1], xi-knot);
      } else {
        return polyeval(coef[0], xi);
      }
    }
  }

  void display() {
    int N = coef.size();
    cout << "Piece-wise polynomial with " << N << " pieces :" << endl;
    for (int i = 0; i<N; ++i) {
      cout << "  Piece " << i+1 << ": " << coef[i].transpose() << endl;
    }
    if (N>1) {
      cout << "  knot = " << knot << endl;
    }
  }
};

#endif
