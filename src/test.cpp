#include "pchip.h"
#include <vector>
#include <iostream>
#include <iomanip>
// #include "Behavior.h"
#include "Vehicle.h"
// #include "roots.h"
#include "polynomial.h"
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "TrajectoryGenerator.h"


using namespace std;
using namespace Eigen;
int main() {
/* Test polynomial functions
  // VectorXd x(7);
  // x << 0, 2, 4, 0, 10, 11, 0;


  VectorXd A = VectorXd::Random(8);
  cout << "Polynomial A :" << endl;
  cout << A.transpose() << endl << endl;

  VectorXd Ad = polyder(A);
  cout << "dA/ds :" << endl;
  cout << Ad.transpose() << endl << endl;

  VectorXd Adi = polyint(Ad);
  cout << "int(dA/ds) :" << endl;
  cout << Adi.transpose() << endl << endl;

  double A_xi = polyval(A, 3);
  cout << "A(3) :" << endl;
  cout << A_xi << endl << endl;

  VectorXd Ad_xi = polyeval(A, 3);
  cout << "[A(3), A'(3), A''(3), A'''(3), ...] :" << endl;
  cout << Ad_xi.transpose() << endl << endl;

  auto x2 = realRoots(x);
  cout << "Real roots of A :" << endl;
  cout << x2.transpose() << endl << endl;

  VectorXd c1(3), c2(3);
  c1 << 2, 1, 0.5;
  c2 << 3, 2, 1;

  PP pp (c1);
  cout << pp.ppval(2) << "  " << pp.ppval(4) << endl << endl;
  cout << pp.ppeval(2).transpose() << endl << endl;
*/


/* // Test pchip
  vector<double> y = {0, 1, 5, 9, 3};
  vector<double> y2 = {0,5,2,0,3};

  pchip pp = pchip(x,y);
  pchip pp2 = pchip(x,y2);

  int N = 20;
  double dx = (x.back() - x.front())/(N-1);

  vector<double> x_i;
  for (int i=0; i<N; ++i) {x_i.push_back(x.front()+i*dx);}

  int idx = -1;
  auto y_i = pp.evaluate(3, &idx);
  auto y_i2 = pp2.evaluate(3);

  for (int i=0; i<4; i++) { cout << y_i[i] << ", "; }
  cout << endl;
  for (int i=0; i<4; i++) { cout << y_i2[i] << ", "; }
  cout << endl;

*/


// Test helpers
/*
  VectorXd c1(VectorXd::Random(4));
  VectorXd c2(VectorXd::Random(1));
  Trajectory traj(c1,c2,3);
  cout << "Trajectory is a ";
  traj.display();
  cout << endl;

  Vector3d s1(traj.state_at(2));
  cout << "Trajectory state_at(2) :" << endl;
  cout << s1.transpose() << endl << endl;

  Vector3d s2(traj.state_at(4));
  cout << "Trajectory state_at(4) :" << endl;
  cout << s2.transpose() << endl << endl;

  cout << "State created from the above two state_at's." << endl;
  State state(s1,s2);
  state.display();
  cout << endl;

  cout << "Trajectories of this state :" << endl;
  auto trajs = state.get_trajectory();
  trajs[0].display();
  trajs[1].display();

  Rectangle rec1(5,2,3.1415/4);
  cout << rec1() << endl << endl;

  Rectangle rec2(3,3,0,1,-1);
  cout << rec1.overlap(rec2) << endl;
*/

  // Vector3d x,y;
  // x << 1,2,3;
  // y << 2,1,3;
  //
  // cout << ((ArrayXXd::Ones(3,3)).colwise() * x.array()).matrix().rowwise() + y.transpose() << endl;
  //
  // vector<double> x1 = {1,2,3};
  // vector<double> y1 = {2,1,3};
  //
  // int i;
  // int j;
  // for (int k=0; k<9; ++k) {
  //   ind2sub(3,k,i,j);
  //   cout << "idx = " << k << " ind2sub : " << "i=" << i << ", j=" << j << " cost : " << x1[i] + y1[j] << endl;
  // }



// Test Vehicle
  Vehicle car(2);
  car.display();

  cout << "Vehicle orientation at 1: ";
  cout << car.orientation(1) << endl;

  cout << endl;

  Vector3d x, y;
  x << 0,15,-0.2;
  y << 6,1,-1;
  car.set_state(State(x,y));

  car.display();
  cout << endl;

  cout << "Vehicle orientation at 0, 1, 2: " << endl;
  cout << car.orientation(0) << endl;
  cout << car.orientation(1) << endl;
  cout << car.orientation(2) << endl;

  cout << endl;
  cout << "Bounding boxes at t=0,1,2:" << endl;
  cout << car.bounding_box(0)() << endl << endl;
  cout << car.bounding_box(1)() << endl << endl;
  cout << car.bounding_box(2)() << endl << endl;

  cout << endl;

  cout << "Locations at t in [0,2] (x, y) " << endl;
  VectorXd t = VectorXd::LinSpaced(20,0,2);
  auto loc = car.location_Eig(t);
  for (int i=0; i<loc.rows(); ++i) {
    cout << loc(i,0) << "   " << loc(i,1) << endl;
  }


  //
  // vector<double> state = {3,2,4};
  // ActiveMode activeMode = ActiveMode("following", state);
  // ActiveMode activeMode2 = ActiveMode("Velocity_Keeping", 20.0);
  //
  // cout << activeMode.mode << endl << activeMode.state[0] << endl;

  // cout << endl << "x" << setw(12)
  //              << "y" << setw(12)
  //              << "y'" << setw(12)
  //              << "y''" << setw(12)
  //              << "y'''" << endl;
  // cout << "---------------------------------------------" << endl;
  // for (int i=0; i<N; ++i) {
  //   cout << x_i[i] << setw(12) << y_i[i][0] << setw(12) << y_i[i][1] << setw(12) << y_i[i][2] << setw(12) << y_i[i][3]<< endl;
  // }
  //
  // cout << endl;
  // cout << pp.coeffs;
  // cout << endl;
  return 0;
}
