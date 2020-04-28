#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE
  const float denom2 = px * px + py * py;
  const float denom = std::sqrt(denom2);
  const float denom3 = denom * denom2;

  // check division by zero
  if (std::abs(px - py) <= 1e-8) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj(0, 0) = px / denom;
  Hj(0, 1) = py / denom;
  Hj(0, 2) = Hj(0, 3) = 0.0f;
  Hj(1, 0) = -py / denom2;
  Hj(1, 1) = px / denom2;
  Hj(1, 2) = Hj(1, 3) = 0.0f;
  Hj(2, 0) = py * (vx * py - vy * px) / denom3;
  Hj(2, 1) = px * (vy * px - vx * py) / denom3;
  Hj(2, 2) = px / denom;
  Hj(2, 3) = py / denom;

  return Hj;
}