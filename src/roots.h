#ifndef ROOTS_H
#define ROOTS_H

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

#include "Eigen-3.3/Eigen/Eigenvalues"
#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace Eigen;
using namespace std;

VectorXd realRoots(VectorXd const & coefs) {
  int N = coefs.size();
  vector<double> a;

  double tol = 1e-10;

  // Count leading zeros (and remove them)
  int num_leading_zeros = 0;
  bool is_leading_zero = true;
  for (int i=0; i<N; ++i) {
    if (coefs(i) < tol && is_leading_zero) {
      ++num_leading_zeros;
    } else {
      is_leading_zero = false;
      a.push_back(coefs(i));
    }
  }

  // Remove trailing zeros
  for (int i=a.size()-1; i>0; --i) {
    if (a[i] < tol) {
      a.pop_back();
    } else {
      break;
    }
  }

  // Form the companion matrix
  N = a.size() - 1;
  MatrixXd C = MatrixXd::Zero(N, N);

  C.block(1,0,N-1,N-1) = MatrixXd::Identity(N-1,N-1);

  for (int i = 0; i<N; ++i) {
    C(i, N-1) = -a[i]/a.back();
  }

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

#endif
