#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::UKF() {
    use_laser_ = true;
    use_radar_ = true;
    x_ = VectorXd(5);
    P_ = MatrixXd(5, 5);
    std_a_ = 1.5;
    std_yawdd_ = 0.6;
    std_laspx_ = 0.15;
    std_laspy_ = 0.15;
    std_radr_ = 0.3;
    std_radphi_ = 0.03;
    std_radrd_ = 0.3;
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    x_ = VectorXd(n_x_);
    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    weights_ = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    int index0 = 0;
    for (index0 = 1; index0 < 2 * n_aug_ + 1; index0++) 
    {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(index0) = weight;
    }
    Q = MatrixXd(2, 2);
    Q << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;
    MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
            0, std_radphi_ * std_radphi_, 0,
            0, 0, std_radrd_ * std_radrd_;
}

UKF::~UKF() 
{

}

void UKF::ProcessMeasurement(MeasurementPackage measurement_package) 
{
    if (!is_initialized_) 
	{
        if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) 
		{
		   	double px = measurement_package.raw_measurements_[0] * cos(measurement_package.raw_measurements_[1]);
            		double py = measurement_package.raw_measurements_[0] * sin(measurement_package.raw_measurements_[1]);
            		double yawd = 0.0;
			x_ << px, py, measurement_package.raw_measurements_[2], measurement_package.raw_measurements_[1], 0;
		} 
		else if (measurement_package.sensor_type_ == MeasurementPackage::LASER) 
		{
            x_ << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1], 0, 0, 0;
        }
		is_initialized_ = true;
        prev_time = measurement_package.timestamp_;   
        return;
    }
	if ((use_radar_ && measurement_package.sensor_type_ == measurement_package.RADAR) ||
        (use_laser_ && measurement_package.sensor_type_ == measurement_package.LASER)) {
        double delta_t = (measurement_package.timestamp_ - prev_time) / 1000000.0;
        prev_time = measurement_package.timestamp_;

        Prediction(delta_t);
        UpdateMeasurement(measurement_package);
    }
}

/*
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (!is_initialized_) {
        // Switch between lidar and radar
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double v = meas_package.raw_measurements_[2];
            double px = rho * cos(phi);
            double py = rho * sin(phi);
            double yawd = 0.0;
            x_ << px, py, v, phi, yawd;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            double px = meas_package.raw_measurements_[0];
            double py = meas_package.raw_measurements_[1];
            x_ << px, py, 0, 0, 0;
        }
        prev_time = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    if ((use_radar_ && meas_package.sensor_type_ == meas_package.RADAR) ||
        (use_laser_ && meas_package.sensor_type_ == meas_package.LASER)) {
        double delta_t = (meas_package.timestamp_ - prev_time) / 1000000.0;
        prev_time = meas_package.timestamp_;

        Prediction(delta_t);
        UpdateMeasurement(meas_package);
    }
}

*/











void UKF::UpdateMeasurement(MeasurementPackage measurement_package) 
{
    VectorXd z;
    MatrixXd z_sig;
    VectorXd z_pred;
    MatrixXd R;
    int n_z;

    if (measurement_package.sensor_type_ == measurement_package.RADAR) 
{
        n_z = n_z_radar_;
        R = R_radar_;
        double rho = measurement_package.raw_measurements_[0];
        double phi = measurement_package.raw_measurements_[1];
        double v = measurement_package.raw_measurements_[2];
        z = VectorXd(n_z);
        z << rho, phi, v;
        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);
	int index2 = 0;
        for (index2 = 0; index2 < 2 * n_aug_ + 1; index2++) {
            double p_x = Xsig_pred_(0, index2);
            double p_y = Xsig_pred_(1, index2);
            double vel = Xsig_pred_(2, index2);
            double yaw = Xsig_pred_(3, index2);
            double v_x = cos(yaw) * vel;
            double v_y = sin(yaw) * vel;
            z_sig(0, index2) = sqrt(p_x * p_x + p_y * p_y);
            z_sig(1, index2) = atan2(p_y, p_x);
            z_sig(2, index2) = (z_sig(0, index2) < 0.0001 ? (p_x * v_x + p_y * v_y) / 0.0001 : (p_x * v_x + p_y * v_y) / z_sig(0, index2));
        }
    } 
	else if (measurement_package.sensor_type_ == measurement_package.LASER) 
	{
        n_z = n_z_lidar_;
        R = R_laser_;

        double px = measurement_package.raw_measurements_[0];
        double py = measurement_package.raw_measurements_[1];

        z = VectorXd(n_z);
        z << px, py;

        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);
	int index3 = 0;
        for (index3 = 0; index3 < 2 * n_aug_ + 1; index3++) 
	{
            z_sig(0, index3) = Xsig_pred_(0, index3);
            z_sig(1, index3) = Xsig_pred_(1, index3);
        }
    }
	z_pred.fill(0.0);
	int index4 = 0;
    for (index4 = 0; index4 < 2 * n_aug_ + 1; index4++) {
        z_pred = z_pred + weights_(index4) * z_sig.col(index4);
    }
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
	int index5 = 0;
    for (index5 = 0; index5 < 2 * n_aug_ + 1; index5++) {
        VectorXd z_difference = z_sig.col(index5) - z_pred;
        S += weights_(index5) * z_difference * z_difference.transpose();
        VectorXd x_difference = Xsig_pred_.col(index5) - x_;
        x_difference(3) = remainder(x_difference(3), 2.0 * M_PI); 
        Tc += weights_(index5) * x_difference * z_difference.transpose();
    }
    S = S + R;
    MatrixXd K = Tc * S.inverse();
    VectorXd z_diff = z - z_pred;
    if (measurement_package.sensor_type_ == measurement_package.RADAR) {
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);
    }
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    VectorXd NIS = z_diff.transpose() * S.inverse() * z_diff;
    cout << " measurement is " << NIS << endl;
}

void UKF::UpdateLidar(MeasurementPackage measurement_package) 
{
    UpdateMeasurement(measurement_package);
}


void UKF::UpdateRadar(MeasurementPackage measurement_package) 
{
    UpdateMeasurement(measurement_package);
}


/*
void UKF::Prediction(double delta_t) {
    VectorXd x_aug = VectorXd(n_aug_);

    x_aug.fill(0.0);
    x_aug.block<5, 1>(0, 0) = x_;

    //create augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug.bottomRightCorner(2, 2) = Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    MatrixXd sqaure_root = sqrt(lambda_ + n_aug_) * A;

    //create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    MatrixXd x_replicated = x_aug.rowwise().replicate(7);
    Xsig_aug.block<7, 1>(0, 0) = x_aug;
    Xsig_aug.block<7, 7>(0, 1) = x_replicated + sqaure_root;
    Xsig_aug.block<7, 7>(0, 8) = x_replicated - sqaure_root;


    // Find State Sigma Points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
	
	
	
	
	
     x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // normalize angle
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI);

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}


*/





 





void UKF::Prediction(double small_t) 
{
    Eigen::VectorXd x_aug = Eigen::VectorXd(7);
    x_aug.fill(0.0);
    x_aug.block<5, 1>(0, 0) = x_;
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(7, 7);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug.bottomRightCorner(2, 2) = Q;
    Eigen::MatrixXd A = P_aug.llt().matrixL();
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(7,15);
    Eigen::MatrixXd x_replicated = x_aug.rowwise().replicate(7);
    Xsig_aug.block<7, 1>(0, 0) = x_aug;
    Xsig_aug.block<7, 7>(0, 1) = x_replicated + (1.732 * A);
    Xsig_aug.block<7, 7>(0, 8) = x_replicated - (1.732 * A);
    Xsig_pred_ = Eigen::MatrixXd(5,15);
	int index1 = 1;
    for (index1 = 0; index1 < 15; index1++) 
	{
        double px_p, py_p;
        if (fabs(Xsig_aug(4, index1)) > 0.001) 
		{
            px_p = Xsig_aug(0, index1) + Xsig_aug(2, index1) / Xsig_aug(4, index1) * (sin(Xsig_aug(3, index1) + Xsig_aug(4, index1) * small_t) - sin(Xsig_aug(3, index1))) + + 0.5 * Xsig_aug(5, index1) * small_t * small_t * cos(Xsig_aug(3, index1));
            py_p = Xsig_aug(1, index1) + Xsig_aug(2, index1) / Xsig_aug(4, index1) * (cos(Xsig_aug(3, index1)) - cos(Xsig_aug(3, index1) + Xsig_aug(4, index1) * small_t)) + 0.5 * Xsig_aug(5, index1) * small_t * small_t * sin(Xsig_aug(3, index1));
        } 
		else 
		{
            px_p = Xsig_aug(0, index1) + Xsig_aug(2, index1) * small_t * cos(Xsig_aug(3, index1)) + 0.5 * Xsig_aug(5, index1) * small_t * small_t * cos(Xsig_aug(3, index1));
            py_p = Xsig_aug(1, index1) + Xsig_aug(2, index1) * small_t * sin(Xsig_aug(3, index1)) + 0.5 * Xsig_aug(5, index1) * small_t * small_t * sin(Xsig_aug(3, index1));
        }

        double v_p;
        double yaw_p;
        double yawd_p;
        v_p = Xsig_aug(2, index1) + Xsig_aug(5, index1) * small_t;

        yaw_p = Xsig_aug(3, index1) + Xsig_aug(4, index1) * small_t + 0.5 * Xsig_aug(6, index1) * small_t * small_t;
        yawd_p = Xsig_aug(4, index1) + Xsig_aug(6, index1) * small_t;

        Xsig_pred_(0, index1) = px_p;
        Xsig_pred_(1, index1) = py_p;
        Xsig_pred_(2, index1) = v_p;
        Xsig_pred_(3, index1) = yaw_p;
        Xsig_pred_(4, index1) = yawd_p;
    }		
    x_.fill(0.0);
	int index2 = 1;
    for (index2 = 0; index2<15; index2++) 
	{
        x_ += weights_(index2) * Xsig_pred_.col(index2);
    }
    P_.fill(0.0);
	int index3 = 1;
    for (index3=0; index3<15; index3++) 
	{
        Eigen::VectorXd x_difference;
		x_difference = Xsig_pred_.col(index3) - x_;
        x_difference(3) = remainder(x_difference(3), 2 * M_PI);
        P_ += weights_(index3) * x_difference * x_difference.transpose();
    }
}
/*
void UpdateLidar_1(MeasurementPackage measurement_package) 
{
    Eigen::VectorXd z;
    Eigen::MatrixXd z_sig;
    Eigen::VectorXd z_pred;
    Eigen::MatrixXd R;
        R = R_laser_;

        z = Eigen::VectorXd(2);
        z << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1];

        z_sig = Eigen::MatrixXd(2, 15);
        z_pred = Eigen::VectorXd(2);
		int index11 = 1;
        for (index11 = 0; index11 < 15; index11++) 
		{
            z_sig(0, index11) = Xsig_pred_(0, index11);
            z_sig(1, index11) = Xsig_pred_(1, index11);
        }
	
    z_pred.fill(0.0);
	int indexx1 = 1;
    for (indexx1 = 0; indexx1 < 15; indexx1++) 
	{
        z_pred = z_pred + weights_(indexx1) * z_sig.col(indexx1);
    }
    Eigen::MatrixXd S = Eigen::MatrixXd(2, 2);
    S.fill(0.0);
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, 2);
    Tc.fill(0.0);
	int indexy1 = 1;
    for (indexy1 = 0; indexy1 < 15; indexy1++) 
	{
        Eigen::VectorXd z_difference;
		z_difference = z_sig.col(indexy1) - z_pred;
        S += weights_(indexy1) * z_difference * z_difference.transpose();
        Eigen::VectorXd x_difference = Xsig_pred_.col(indexy1) - x_;
        x_difference(3) = remainder(x_difference(3), 2.0 * M_PI);
        Tc += weights_(indexy1) * x_difference * z_difference.transpose();
    }
    S = S + R;
    Eigen::MatrixXd K;
	K = Tc * S.inverse();
    Eigen::VectorXd z_difference;
	z_difference= z - z_pred;
    if (measurement_package.sensor_type_ == measurement_package.RADAR) 
	{
        z_difference(1) = remainder(z_difference(1), 2.0 * M_PI);
    }
    x_ = x_ + K * z_difference;
    P_ = P_ - K * S * K.transpose();
    Eigen::VectorXd NIS;
	NIS = z_difference.transpose() * S.inverse() * z_difference;
    cout << " measurement : " << NIS << endl;
}

void UpdateRadar_1(MeasurementPackage measurement_package) 
{
	Eigen::VectorXd z;
    Eigen::MatrixXd z_sig;
    Eigen::VectorXd z_pred;
    Eigen::MatrixXd R;
        R = R_radar_;
        z = Eigen::VectorXd(3);
        z << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1], measurement_package.raw_measurements_[2];
        z_sig = Eigen::MatrixXd(3, 15);
        z_pred = Eigen::VectorXd(3);
	int index1 = 1;
	
        for (index1=0; index1<15; index1++) 
		{
            z_sig(0, index1) = sqrt(Xsig_pred_(0, index1) * Xsig_pred_(0, index1) + Xsig_pred_(1, index1) * Xsig_pred_(1, index1));
            z_sig(1, index1) = atan2(Xsig_pred_(1, index1), Xsig_pred_(0, index1));
            z_sig(2, index1) = (z_sig(0, index1) < 0.0001 ? (Xsig_pred_(0, index1) * cos(Xsig_pred_(3, index1))*Xsig_pred_(2, index1) + Xsig_pred_(1, index1) * sin(Xsig_pred_(3, index1))*Xsig_pred_(2, index1)) / 0.0001 : (Xsig_pred_(0, index1) * cos(Xsig_pred_(3, index1))*Xsig_pred_(2, index1) + Xsig_pred_(1, index1) * sin(Xsig_pred_(3, index1))*Xsig_pred_(2, index1)) / z_sig(0, index1));
        }
    z_pred.fill(0.0);
	int indexz = 1;
    for (indexz = 0; indexz < 15; indexz++) 
	{
        z_pred = z_pred + weights_(indexz) * z_sig.col(indexz);
    }
    Eigen::MatrixXd S = Eigen::MatrixXd(3, 3);
    S.fill(0.0);
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, 3);
    Tc.fill(0.0);
	int index5 = 1;
    for (index5 = 0; index5 < 15; index5++) 
	{
        Eigen::VectorXd z_difference;
	z_difference = z_sig.col(index5) - z_pred;
        S += weights_(index5) * z_difference * z_difference.transpose();
        Eigen::VectorXd x_difference = Xsig_pred_.col(index5) - x_;
        x_difference(3) = remainder(x_difference(3), 2.0 * M_PI);
        Tc += weights_(index5) * x_difference * z_difference.transpose();
    }
    S = S + R;
    Eigen::MatrixXd K;
	K = Tc * S.inverse();
    Eigen::VectorXd z_difference;
	z_difference= z - z_pred;
    if (measurement_package.sensor_type_ == measurement_package.RADAR) 
	{
        z_difference(1) = remainder(z_difference(1), 2.0 * M_PI);
    }
    x_ = x_ + K * z_difference;
    P_ = P_ - K * S * K.transpose();
    Eigen::VectorXd NIS;
	NIS = z_difference.transpose() * S.inverse() * z_difference;
    cout << " measurement : " << NIS << endl;
}*/
