#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}


/**
 * Programming assignment functions: 
 */

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  // predict sigma points
  for (int i = 0; i < Xsig_aug.cols(); i++) {
    const float px = Xsig_aug(0, i);
    const float py = Xsig_aug(1, i);
    const float v = Xsig_aug(2, i);
    const float yaw = Xsig_aug(3, i);
    const float yaw_rate = Xsig_aug(4, i);
    const float nu_accel = Xsig_aug(5, i);
    const float nu_yaw_accel = Xsig_aug(6, i);
    VectorXd vec(n_x);
    VectorXd noise_vec(n_x);
    noise_vec << 0.5 * delta_t * delta_t * std::cos(yaw) * nu_accel,
                 0.5 * delta_t * delta_t * std::sin(yaw) * nu_accel,
                 delta_t * nu_accel, 0.5 * delta_t * delta_t * nu_yaw_accel,
                 delta_t * nu_yaw_accel;
    // avoid division by zero                          
    if (std::abs(yaw_rate) <= 1e-10) {
      vec << v * std::cos(yaw) * delta_t, v * std::sin(yaw) * delta_t, 0, 0, 0;
    } else {
      vec << (v / yaw_rate) * (std::sin(yaw + yaw_rate * delta_t) - std::sin(yaw)),
             (v / yaw_rate) * (-std::cos(yaw + yaw_rate * delta_t) + std::cos(yaw)),
             0, yaw_rate * delta_t, 0;
    }
    // write predicted sigma points into right column
    Xsig_pred.col(i) = Xsig_aug.col(i).head(n_x) + vec + noise_vec;
  }

  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}