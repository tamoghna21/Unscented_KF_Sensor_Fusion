#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  //std_a_ = 3;
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 1.5;
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  //std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //std_radrd_ = 0.1;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  is_initialized_ = false;
  time_us_ = 0;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  
  //set measurement dimension; 
  n_z_radar_ = 3; //radar can measure r, phi and r_dot
  n_z_lidar_ = 2; //lidar can measure px and py
  
  // augmented mean vector
  x_aug_ = VectorXd(n_aug_);
  
  // augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
  
  // augmented state covariance matrix
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
  
  //predicted state vector
  x_pred_ = VectorXd(n_x_);
  
  //Predicted state covariance matrix;
  P_pred_ = MatrixXd(n_x_,n_x_);
  
  //create sigma points matrix in measurement space
  Zsig_radar_= MatrixXd(n_z_radar_,2*n_aug_+1);
  Zsig_lidar_= MatrixXd(n_z_lidar_,2*n_aug_+1);
  
  // H_ for laser
  H_ = MatrixXd(n_z_lidar_,n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
        
  //NIS
  nis_radar = 0;
  nis_lidar = 0;
  
  //vector< double > nis_radar_vals;
  //vector< double > nis_lidar_vals;
  
  //R matrices
  R_laser_ = MatrixXd(n_z_lidar_,n_z_lidar_);
  R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
  R_laser_ << std_laspx_*std_laspx_,0,
              0,std_laspy_*std_laspy_;
  R_radar_ << std_radr_*std_radr_,0,0,
              0, std_radphi_*std_radphi_,0,
			  0,0,std_radrd_*std_radrd_;
  
  //Weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for(int i=1; i<2*n_aug_+1; i++){  //2n+1 weights
	double weight = 0.5/(lambda_ + n_aug_);
	weights_(i) = weight;  
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if ((use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) || (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)){
  	/**
  	TODO:

  	Complete this function! Make sure you switch between lidar and radar
  	measurements.
  	*/
  	/*************************************************************************
  	* Initialization
  	**************************************************************************/
  	if (!is_initialized_){
		// first measurement
		cout << "UKF initialized: " << endl;
		x_ << 1, 1, 1, 1, 1;
	
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
			double ro = meas_package.raw_measurements_[0];
			double theta = meas_package.raw_measurements_[1];
			double ro_dot = meas_package.raw_measurements_[2];
			x_ << ro * cos(theta), ro * sin(theta), 0, 0, 0;
		
			P_  <<0.8,0,0,0,0,
        	0,0.8,0,0,0,
			0,0,0.8,0,0,
			0,0,0,1,0,
			0,0,0,0,1;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],0,0,0;
		
			P_  <<0.8,0,0,0,0,
        	0,0.8,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,1;
		}
		time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		return;
  	}
  	/*************************************************************************
  	*  Create Sigma points
  	**************************************************************************/
  
  	//create augmented sigma vector
  	x_aug_ << x_(0), x_(1), x_(2), x_(3), x_(4), 0, 0;
  
  	//create augmented covariance matrix
  	P_aug_.fill(0.0);
  	P_aug_.topLeftCorner(n_x_,n_x_) = P_;
  	P_aug_(n_x_,n_x_) = std_a_ * std_a_;
  	P_aug_(n_x_+1,n_x_+1) = std_yawdd_ * std_yawdd_;
  	
  
  	//Calculate square root of P_aug_
  	MatrixXd A = P_aug_.llt().matrixL();
  
  	//create square root matrix
  	MatrixXd L = sqrt(lambda_ + n_aug_)*A;
  
  	//create augmented sigma points
  	Xsig_aug_.block(0,0,n_aug_,1) = x_aug_;
  	Xsig_aug_.block(0,1,n_aug_,n_aug_) = x_aug_.rowwise().replicate(n_aug_) + L;
  	Xsig_aug_.block(0,n_aug_+1,n_aug_,n_aug_) = x_aug_.rowwise().replicate(n_aug_) - L;
  
  
  	/*************************************************************************
  	*  Prediction
  	**************************************************************************/
  	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt in seconds
  	time_us_ = meas_package.timestamp_;
  
  	Prediction(dt);
  
  	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
		UpdateRadar(meas_package);
  	}
  	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
		UpdateLidar(meas_package);
  	}
  } //closing bracket for use _aser or use_radar
  
}//End ProcessMeasurement

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  for (int i =0; i< 2*n_aug_+1; i++){
	  double p_x = Xsig_aug_(0,i);
	  double p_y = Xsig_aug_(1,i);
	  double v = Xsig_aug_(2,i);
	  double yaw = Xsig_aug_(3,i);
	  double yawd = Xsig_aug_(4,i);
	  double nu_a = Xsig_aug_(5,i);
	  double nu_yawdd = Xsig_aug_(6,i);
	  
	  //predicted state values
	  double px_p, py_p;
	  
	  //CTRV model
	  //avoid division by zero
	  if(fabs(yawd)> 0.001){
		  px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw));
		  py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t));
	  }
	  else {
		  px_p = p_x + v*delta_t*cos(yaw);
		  py_p = p_y + v*delta_t*sin(yaw);
	  }
	  
	  double v_p = v;
	  double yaw_p = yaw + yawd*delta_t;
	  double yawd_p = yawd;
	  
	  //add noise
	  px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
	  py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
	  v_p = v_p + nu_a*delta_t;
	  
	  yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
	  yawd_p = yawd_p + nu_yawdd*delta_t;
	  
	  //write predicted sigma point into right column
	  Xsig_pred_(0,i) = px_p;
	  Xsig_pred_(1,i) = py_p;
	  Xsig_pred_(2,i) = v_p;
	  Xsig_pred_(3,i) = yaw_p;
	  Xsig_pred_(4,i) = yawd_p;
	  
    }
	
    //predict state mean
	x_.fill(0.0);
	for(int i=0; i<2*n_aug_+1;i++){ //iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}
	
	//predicted state covariance matrix
	P_.fill(0.0);
	for (int i=0; i<2*n_aug_+1;i++){  //iterate over sigma points
		
		//state difference_type
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		
		//angle normalization
		while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
		while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
		
		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
		
	}
}
	

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd z = VectorXd(n_z_lidar_);
  z << meas_package.raw_measurements_;
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_ .transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ +(K * y);
  long x_zize = x_.size();
  MatrixXd I = MatrixXd::Identity(x_zize, x_zize);
  P_ = (I - K * H_) * P_;
  
  //calculate NIS
  nis_lidar = y.transpose() * Si * y;
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //transform sigma points into measurement space
  VectorXd z = VectorXd(n_z_radar_);
  z << meas_package.raw_measurements_;
  
  for (int i =0; i< 2*n_aug_ + 1; i++){
	double px_p = Xsig_pred_(0,i);
	double py_p = Xsig_pred_(1,i);
	double v_p = Xsig_pred_(2,i);
	double yaw_p = Xsig_pred_(3,i);
	
	double v1 = cos(yaw_p)*v_p;
	double v2 = sin(yaw_p)*v_p;
	
	//measurement model
	Zsig_radar_(0,i) = sqrt(px_p*px_p + py_p*py_p);                     //r
	Zsig_radar_(1,i) = atan2(py_p, px_p);                               //phi
	Zsig_radar_(2,i) = (px_p*v1 + py_p*v2) / sqrt(px_p*px_p + py_p*py_p);//r_dot
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_ + 1; i++ ){
	z_pred = z_pred +  weights_(i) * Zsig_radar_.col(i);
	
  }
  
  MatrixXd Tc = MatrixXd(n_x_,n_z_radar_);
  Tc.fill(0.0);
  
  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for(int i=0; i<2*n_aug_ + 1; i++ ){
    // residual
	VectorXd z_diff = Zsig_radar_.col(i) - z_pred;

	//angle normalization
	while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	
	S = S + weights_(i) * z_diff * z_diff.transpose();
	
	//state difference
	VectorXd x_diff = Xsig_pred_.col(i) - x_ ;
	//angle normalization
	while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
	while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
	
	Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  S = S + R_radar_;
  
  //residual
  VectorXd z_diff = z - z_pred;
  
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)< -M_PI) z_diff(1)+=2.*M_PI;
  
  //Kalman gain K
  MatrixXd K = Tc * S.inverse();
  
  x_ = x_ + K * z_diff;
  P_  = P_  - K*S*K.transpose();
  
  //calculate NIS
  nis_radar = z_diff.transpose() * S.inverse() * z_diff;
  
}
