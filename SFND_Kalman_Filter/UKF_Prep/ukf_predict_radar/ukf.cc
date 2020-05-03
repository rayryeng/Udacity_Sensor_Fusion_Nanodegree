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

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  // double weight_0 = lambda/(lambda+n_aug);
  // double weight = 0.5/(lambda+n_aug);
  // weights(0) = weight_0;

  // for (int i=1; i<2*n_aug+1; ++i) {  
  //   weights(i) = weight;
  // }
  weights.fill(1 / (2 * (lambda + n_aug)));
  weights(0) = lambda / (lambda + n_aug);

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  /**
   * Student part begin
   */

  // transform sigma points into measurement space
  Zsig.row(0) = ((Xsig_pred.row(0).array() * Xsig_pred.row(0).array()) +
                 (Xsig_pred.row(1).array() * Xsig_pred.row(1).array())).sqrt();
  //Zsig.row(1) = (Xsig_pred.row(1).array() / Xsig_pred.row(0).array()).atan();
  for (int i = 0; i < Zsig.cols(); i++) {
    Zsig(1, i) = std::atan2(Xsig_pred(1, i), Xsig_pred(0, i));
  }  
  Zsig.row(2) = ((Xsig_pred.row(0).array() * Xsig_pred.row(2).array() * Xsig_pred.row(3).array().cos()) +
                 (Xsig_pred.row(1).array() * Xsig_pred.row(2).array() * Xsig_pred.row(3).array().sin())) / Zsig.row(0).array();
  // calculate mean predicted measurement
  z_pred = Zsig * weights;
  // calculate innovation covariance matrix S
  MatrixXd R(3, 3);
  R << std_radr * std_radr, 0, 0,
       0, std_radphi * std_radphi, 0,
       0, 0, std_radrd * std_radrd;
  S = (Zsig.array().colwise() - z_pred.array());
  S = S * weights.asDiagonal() * S.transpose() + R;
  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}