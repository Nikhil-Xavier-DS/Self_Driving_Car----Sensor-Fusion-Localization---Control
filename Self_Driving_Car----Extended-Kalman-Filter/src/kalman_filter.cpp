#include "kalman_filter.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() 
{
	
}

KalmanFilter::~KalmanFilter() 
{
	
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  x_ = x_in; P_ = P_in; F_ = F_in; H_ = H_in; R_ = R_in; Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  VectorXd z_pred, y;
  MatrixXd Ht, S, Si, K, I;
  long x_size;
  z_pred = H_ * x_;
  z_pred = H_*x_;
  y = z - z_pred;
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  K = P_ * Ht * Si;
  x_ = x_ + K * y;
  x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  VectorXd z_pred(3);
  double rho, rho_dot, phi;
  rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / std::max(rho, 0.0001);
  phi = atan2(x_(1), x_(0));
if(rho < 0.001)
{
 phi = 0;
 rho_dot = 0;
}
else
{
  phi = atan2(x_(1), x_(0));
  rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / std::max(rho, 0.0001);
}
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  while (y(1) > M_PI) 
 {
	  y(1) -= 2 * M_PI;
 }
  while (y(1) < -M_PI) 
 {
	  y(1) += 2 * M_PI;
 }
  MatrixXd Ht, S, Si, K, I;
  long x_size;
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_; Si = S.inverse(); K = P_ * Ht * Si; x_ = x_ + K * y; x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
