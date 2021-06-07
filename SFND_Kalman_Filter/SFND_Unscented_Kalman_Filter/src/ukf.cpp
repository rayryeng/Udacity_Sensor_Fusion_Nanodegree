#include "ukf.h"
#include <Eigen/Dense>
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    // set the state with the initial location and zero velocity
    // Must determine what sensor this is first
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
            0, std_laspy_ * std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    } else if (meas_package.sensor_type_ ==
               MeasurementPackage::SensorType::RADAR) {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rhodot = meas_package.raw_measurements_(2);
      const double x = rho * cos(phi);
      const double y = rho * sin(phi);
      const double vx = rhodot * cos(phi);
      const double vy = rhodot * sin(phi);
      const double v = rhodot; // std::sqrt(vx * vx + vy * vy);
      x_ << x, y, v, rho, rhodot;
      P_ << std_radr_* std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, std_radrd_ * std_radrd_, 0, 0,
            0, 0, 0, std_radphi_ * std_radphi_, 0,
            0, 0, 0, 0, std_radphi_ * std_radphi_;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  const float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Predict the next states and covariance matrix
  Prediction(dt);

  // Update the next states and covariance matrix
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ ==
             MeasurementPackage::SensorType::RADAR) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Step #1 - Create augmented mean vector, augmented state covariance
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.setZero(n_aug_);
  x_aug.head(5) = x_;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug.bottomRightCorner(2, 2) << std_a_ * std_a_, 0, 0,
      std_yawdd_ * std_yawdd_;

  // Step #2 - Create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // Step #3 - Create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.middleCols(1, n_aug_) =
      (std::sqrt(lambda_ + n_aug_) * A.array()).colwise() + x_aug.array();
  Xsig_aug.middleCols(n_aug_ + 1, n_aug_) =
      (-std::sqrt(lambda_ + n_aug_) * A.array()).colwise() + x_aug.array();

  // Step #4 - Predict sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < Xsig_aug.cols(); i++) {
    const float px = Xsig_aug(0, i);
    const float py = Xsig_aug(1, i);
    const float v = Xsig_aug(2, i);
    const float yaw = Xsig_aug(3, i);
    const float yaw_rate = Xsig_aug(4, i);
    const float nu_accel = Xsig_aug(5, i);
    const float nu_yaw_accel = Xsig_aug(6, i);
    VectorXd vec(n_x_);
    VectorXd noise_vec(n_x_);
    noise_vec << 0.5 * delta_t * delta_t * std::cos(yaw) * nu_accel,
        0.5 * delta_t * delta_t * std::sin(yaw) * nu_accel, delta_t * nu_accel,
        0.5 * delta_t * delta_t * nu_yaw_accel, delta_t * nu_yaw_accel;
    // avoid division by zero
    if (std::abs(yaw_rate) <= 1e-10) {
      vec << v * std::cos(yaw) * delta_t, v * std::sin(yaw) * delta_t, 0, 0, 0;
    } else {
      vec << (v / yaw_rate) *
                 (std::sin(yaw + yaw_rate * delta_t) - std::sin(yaw)),
          (v / yaw_rate) *
              (-std::cos(yaw + yaw_rate * delta_t) + std::cos(yaw)),
          0, yaw_rate * delta_t, 0;
    }
    // write predicted sigma points into right column
    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + vec + noise_vec;
  }

  // Step #5 - Now predict state mean and covariance
  x_ = Xsig_pred_ * weights_;

  P_ = (Xsig_pred_.array().colwise() - x_.array());
  P_ = P_ * weights_.asDiagonal() * P_.transpose();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // Note that the LiDAR noise profile is linear, so we can simply
  // use the standard linear Kalman filter here
  // Taken directly from the previous assignments

  // New - define measurement matrix
  MatrixXd H;
  H.setZero(2, n_x_);
  H(0, 0) = H(1, 1) = 1;  // Select out the position elements only
  VectorXd z_pred = H * x_;
  const VectorXd z = meas_package.raw_measurements_;

  // Calculate residual vector y
  const VectorXd y = z - z_pred;

  // New - define measurement noise matrix
  R_.setZero(2, 2);
  R_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // Create innovation covariance matrix S
  MatrixXd S = H * P_ * H.transpose() + R_;

  // Create Kalman gain matrix K
  MatrixXd K = P_ * H.transpose() * S.inverse();

  // Create new estimate for states and covariance
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H) * P_;

  // Calculate NIS for LiDAR
  NIS_lidar_ = y.transpose() * S.inverse() * y;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // Step #1 - Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);

  // Step #2 - Transform sigma points into measurement space
  Zsig.row(0) = ((Xsig_pred_.row(0).array() * Xsig_pred_.row(0).array()) +
                 (Xsig_pred_.row(1).array() * Xsig_pred_.row(1).array()))
                    .sqrt();
  for (int i = 0; i < Zsig.cols(); i++) {
    Zsig(1, i) = std::atan2(Xsig_pred_(1, i), Xsig_pred_(0, i));
  }
  Zsig.row(2) = ((Xsig_pred_.row(0).array() * Xsig_pred_.row(2).array() *
                  Xsig_pred_.row(3).array().cos()) +
                 (Xsig_pred_.row(1).array() * Xsig_pred_.row(2).array() *
                  Xsig_pred_.row(3).array().sin())) /
                Zsig.row(0).array();

  // Step #3 - Create and run final update method
  // mean predicted measurement
  const int n_z = 3;
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // Step #4 - Calculate mean predicted measurement
  z_pred = Zsig * weights_;

  // Step #5 - Create measurement covariance matrix R and
  // calculate innovation covariance matrix S
  R_.setZero(n_z, n_z);
  R_ << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0,
      std_radrd_ * std_radrd_;

  S = (Zsig.array().colwise() - z_pred.array());
  S = S * weights_.asDiagonal() * S.transpose() + R_;

  // Step #6 - Calculate Cross Correlation Matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc = (Xsig_pred_.array().colwise() - x_.array()).matrix() *
       weights_.asDiagonal() *
       (Zsig.array().colwise() - z_pred.array()).matrix().transpose();

  // Step #7 - Calculate Kalman gain K
  const MatrixXd K = Tc * S.inverse();

  // Step #8 - Update state mean and covariance matrix
  const VectorXd z = meas_package.raw_measurements_;
  const VectorXd y = z - z_pred;
  x_ = x_ + (K * y);
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for Radar
  NIS_radar_ = y.transpose() * S.inverse() * y;
}