#include "pchip.h"
#include <vector>
#include <iostream>
#include <iomanip>

using namespace std;

int main() {
  vector<double> x = {0, 2, 4, 7, 10};
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
  auto y_i2 = pp2.evaluate(3, &idx);

  for (int i=0; i<4; i++) { cout << y_i[i] << ", "; }
  cout << endl;
  for (int i=0; i<4; i++) { cout << y_i2[i] << ", "; }
  cout << endl;

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
