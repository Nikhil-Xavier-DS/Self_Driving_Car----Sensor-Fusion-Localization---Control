#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;






long long prev_time;






  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;




    MatrixXd Q;
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    int n_z_radar_ = 3;
    int n_z_lidar_ = 2;
    //n_z_radar_ = 3;
    //n_z_lidar_ = 2;
	

  UKF();
  virtual ~UKF();
  void ProcessMeasurement(MeasurementPackage meas_package);
  void Prediction(double delta_t);
  void UpdateLidar(MeasurementPackage meas_package);
    void UpdateRadar(MeasurementPackage measurement_package);
    void GenerateStateSigmaPoints(double small_t);
    void PredictMeanAndCovariance();
    void UpdateMeasurement(MeasurementPackage measurement_package);
};

#endif
