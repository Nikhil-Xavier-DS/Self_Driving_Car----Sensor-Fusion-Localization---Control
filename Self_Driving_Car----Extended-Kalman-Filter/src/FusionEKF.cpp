#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
FusionEKF::FusionEKF() 
{
  noise_ax = 10;
  noise_ay = 5;
  is_initialized_ = false;
  previous_timestamp_ = 0;
  R_laser_ = MatrixXd(2, 2); R_radar_ = MatrixXd(3, 3); H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  R_laser_ << 0.0225, 0,
        0, 0.0225;
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 

  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
}

FusionEKF::~FusionEKF() 
{
	
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
if (!is_initialized_) 
  {
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
    float rho, phi, rho_dot;
    rho = measurement_pack.raw_measurements_[0]; phi = measurement_pack.raw_measurements_[1];rho_dot = measurement_pack.raw_measurements_[1];
    ekf_.x_(0) = rho     * cos(phi);
    ekf_.x_(1) = rho     * sin(phi);      
    ekf_.x_(2) = rho_dot * cos(phi);
    ekf_.x_(3) = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
	  ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
  }
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
}
   float dt, dt_2, dt_3, dt_4;
   dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
   dt_2 = dt * dt; dt_3 = dt_2 * dt; dt_4 = dt_3 * dt;
   ekf_.F_(0, 2) = dt; ekf_.F_(1, 3) = dt;
   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
   			      0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
   			      dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
   			      0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
  ekf_.Predict();
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    ekf_.R_ = R_radar_; ekf_.H_ = tools.CalculateJacobian(ekf_.x_); ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else 
  {
    ekf_.R_ = R_laser_; ekf_.H_ = H_laser_; ekf_.Update(measurement_pack.raw_measurements_);
  }
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
