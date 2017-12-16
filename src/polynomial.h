#ifndef POLYNOMIAL_FUNCTIONS_H
#define POLYNOMIAL_FUNCTIONS_H

/*
Polynomial definision : vector of coeficients where the index is the same as
the power. Example :

a = (a0, a1, a2, a3, ..., an)
A(x) = a(i)*x^i
*/

#include <math.h>
#include <vector>
#include <iostream>
#include <cassert>

#include "Eigen-3.3/Eigen/Eigenvalues"
#include "Eigen-3.3/Eigen/Core"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using std::vector;


/* polyder : compute the derivative of a polynomial. */
VectorXd polyder(VectorXd const& coef, bool same_size = false) {
  int N = coef.size();
  if (N==1) return VectorXd::Zero(1);

  if (same_size) {
    VectorXd coefd = VectorXd::Zero(N);
    coefd.head(N-1) = coef.tail(N-1).cwiseProduct(VectorXd::LinSpaced(N-1,1,N-1));
    return coefd;
  } else {
    return coef.tail(N-1).cwiseProduct(VectorXd::LinSpaced(N-1,1,N-1));
  }
};

/* polyint : integrate a polynomial with constant of integration C. */
VectorXd polyint(VectorXd const& coef, double C = 0) {
  int N = coef.size();
  VectorXd coefi = VectorXd::Zero(N+1);
  coefi.tail(N) = coef.cwiseQuotient(VectorXd::LinSpaced(N,1,N));
  coefi(0) = C;
  return coefi;
};

VectorXd polymult(VectorXd const& coef1, VectorXd const& coef2) {
  // Note, this is using brute force multiplication.
  int N1 = coef1.size();
  int N2 = coef2.size();
  VectorXd coef = VectorXd::Zero(N1+N2-1);
  for (size_t i=0; i<N1; ++i) {
    for (size_t j=0; j<N2; ++j) {
      coef(i+j) += coef1(i) * coef2(j);
    }
  }
  return coef;
}

/* polyval : evaluate a polynomial at xi */
double polyval(VectorXd const& coef, double xi) {
  int N = coef.size();
  if (N==1) return coef(0);

  double yi = coef(N-1);
  for (int i=N-2; i>=0; --i) {
    yi = (yi*xi + coef(i));
  }
  return yi;
};

/* POLYEVAL Evaluate all derivatives of a polynomial at the point x0

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
are simply A_x0(i) = A_d(i) / i!;

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
VectorXd polyeval(VectorXd const& coef, double xi) {

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

/* realRoots - Return the real roots of a polynomial.

Polynomial definision : vector of coeficients where the index is the same as
the power. Example :

a = (a0, a1, a2, a3, ..., an)
A(x) = a(i)*x^i

Note, this program is primarily insterested in returning the unique real.
However, the array returned will not necessarily have only unique values. For
example, if the polynomial has multiple leading zeros, then only one 0 root will
be returned, not all of them.

Based on Matlab's roots.m function() in that it computes the roots using the
companion matrix. However, this function does not check for "virtual" infinities
like Matlab.
*/
VectorXd realRoots(VectorXd const & coefs) {
  bool DEBUG = false;
  if (DEBUG) std::cout << "---------------- realRoots() --------------------" << std::endl;
  if (DEBUG) std::cout << "   Coefs : " << coefs.transpose() << std::endl;
  int N = coefs.size();

  double tol = 1e-10;

  if (N==1 || (coefs.array().abs() < tol).all()) {
    // The polynomial is constant or all zeros, therefore there are no roots!
    return VectorXd::Zero(0);
  }

  vector<double> a;

  // Count leading zeros (and remove them)
  int num_leading_zeros = 0;
  bool is_leading_zero = true;
  for (int i=0; i<N; ++i) {
    if (fabs(coefs(i)) < tol && is_leading_zero) {
      ++num_leading_zeros;
    } else {
      is_leading_zero = false;
      a.push_back(coefs(i));
    }
  }

  // Remove trailing zeros
  for (int i=a.size()-1; i>0; --i) {
    if (fabs(a[i]) < tol) {
      a.pop_back();
    } else {
      break;
    }
  }

  // If there is only one non-zero value at this point, then there is a root at
  // 0. Ex. coef = (0,0,3,0) => A(x) = 3x^2;
  if (a.size()<2) return VectorXd::Zero(1);

  if (DEBUG) std::cout << "   Coef after removing trailing/leading zeros : " << std::endl;
  if (DEBUG) {
    std::cout << "      ";
    for (int i=0; i<a.size(); ++i) std::cout << a[i] << " ";
    std::cout << std::endl;
  }
  // Form the companion matrix
  N = a.size() - 1;
  MatrixXd C = MatrixXd::Zero(N, N);

  C.block(1,0,N-1,N-1) = MatrixXd::Identity(N-1,N-1);

  for (int i = 0; i<N; ++i) {
    C(i, N-1) = -a[i]/a.back();
  }

  if (DEBUG) std::cout << "   Companion matrix : " << std::endl;
  if (DEBUG) std::cout << C << std::endl;

  // Compute the eigenvalues, which are the polynomial roots.
  VectorXcd V = C.eigenvalues();

  // Keep real roots, and add in any zero roots caused by leading zeros
  auto Vi = V.imag().array().abs();
  auto Vr = V.real();

  vector<double> roots;
  for (int i=0; i<Vi.size(); ++i) {
    if (Vi(i) < tol) {
      roots.push_back(Vr(i));
    }
  }

  // If there was a leading zero then add on a 0 root.
  if (num_leading_zeros) roots.push_back(0);

  // Convert the roots vector to a VectorXd
  VectorXd r(roots.size());
  for (int i=0; i<roots.size(); ++i) {
    r(i) = roots[i];
  }
  return r;
}

/* PP2 : Very simple piece-wise polynomial class for only 2 pieces. */
class PP2 {
public:
  vector<VectorXd> coef;
  double knot;

  PP2() : coef({VectorXd::Zero(3)}), knot(1e300) {}
  PP2(VectorXd coef1) : coef({coef1}), knot(1e300) {}
  PP2(VectorXd coef1, VectorXd coef2, double k) : coef({coef1, coef2}), knot(k) {}

  double ppval(double xi) const {
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

  VectorXd ppeval(double xi) const {
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

  void display() const {
    int N = coef.size();
    std::cout << "Piece-wise polynomial with " << N << " pieces :" << std::endl;
    for (int i = 0; i<N; ++i) {
      std::cout << "  Piece " << i+1 << ": " << coef[i].transpose() << std::endl;
    }
    if (N>1) {
      std::cout << "  knot = " << knot << std::endl;
    }
  }
};


/* PP : Piece-wise polynomial class any number of peices */
class PP {
private:
  template <typename T>
  T pp_eval_at(typename std::function<T(VectorXd,double)> f, double& x_i) const {

    auto it = std::lower_bound(knot.begin(),knot.end(),x_i);
    int ii = std::max( int(it-knot.begin())-1, 0);
    double dx = x_i - knot[ii];

    return f(coef[ii], dx);
  }

public:
  vector<VectorXd> coef;
  vector<double> knot;

  PP() : coef({VectorXd::Zero(3)}), knot({1e300,1e300}) {};
  PP(vector<VectorXd> coefs, vector<double> knots) : coef(coefs), knot(knots) {};

  VectorXd ppeval(double& x_i) const {
    return pp_eval_at<VectorXd>(polyeval, x_i);
  }
  double ppval(double& x_i) const {
    return pp_eval_at<double>(polyval, x_i);
  }

  PP ppder() const {
    vector<VectorXd> coef_n(coef.size());
    for (int i=0; i<coef.size(); ++i) {
      coef_n[i] = polyder(coef[i]);
    }
    // std::transform(coef.begin(), coef.end(), coef_n.begin(), polyder);
    return PP(coef_n, knot);
  }

  PP ppint(VectorXd *C = 0) const {
    VectorXd K;
    if (C) {
      K = *C;
      assert(coef.size() == K.size());
    } else {
      K = VectorXd::Zero(coef.size());
    }

    vector<VectorXd> coef_n(coef.size());
    for (int i=0; i<coef.size(); ++i) {
      coef_n[i] = polyint(coef[i], K(0));
    }
    // std::transform(coef.begin(), coef.end(), coef_n.begin(), K.data(), polyint);
    return PP(coef_n, knot);
  }

  PP ppmult(PP& pp2) const {
    // The two polynomials should have the same knots! This could be changed
    // with fancier programming and more time.
    assert(knot.size() == pp2.knot.size());
    for (int i = 0; i<knot.size(); ++i) assert(knot[i]==pp2.knot[i]);

    vector<VectorXd> coef_n(coef.size());
    std::transform(coef.begin(), coef.end(), coef_n.begin(), pp2.coef.begin(), polymult);
    return PP(coef_n, knot);
  }

};

#endif
