#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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
  std_a_ = 3.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 7;
  //initialized state is false
  is_initialized_ = false;

  //state dimension
  n_x_ = 5;
  
  //augmented state dimension
  n_aug_ = 7;

  //initilize lambda value
  lambda_ = 3-n_aug_;

  //initilize the  state
  x_ = VectorXd::Zero(n_x_);

  //initilize the augmented state

  x_aug_ = VectorXd::Zero(n_aug_);

  //initilize the covariance matrix

  P_ = MatrixXd::Zero(n_x_,n_x_);

  P_aug_ = MatrixXd::Zero(n_aug_,n_aug_);

  Xsig_pred_ = MatrixXd::Zero(n_x_,2*n_aug_+1);

  weights_ = VectorXd::Zero(n_aug_*2+1);

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
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if(!is_initialized_){
    if(meas_package.sensor_type_== MeasurementPackage::LASER){
      double posx = static_cast<double>(meas_package.raw_measurements_(0));
      double posy = static_cast<double>(meas_package.raw_measurements_(1));
      double phi = atan2(posy,posx);
      x_<< posx, posy, 0, phi ,0;

    }
    else{
      double rho = static_cast<double>(meas_package.raw_measurements_(0));
      double phi = static_cast<double>(meas_package.raw_measurements_(1));
      double rho_dot = static_cast<double>(meas_package.raw_measurements_(2));
      double posx = rho*cos(phi);
      double posy = rho*sin(phi);
      x_<< posx, posy, rho_dot, phi,0;
      //x_<< posx, posy, 0, 0,0;
    }
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  double dt = ((double)(meas_package.timestamp_-previous_timestamp_))/1000000.0;
  previous_timestamp_ = meas_package.timestamp_;
  
  while(dt>0.1){
    double des =  0.05;
    Prediction(des);
    dt-=des;
  }

  Prediction(dt);

  if(use_laser_&&meas_package.sensor_type_== MeasurementPackage::LASER){
    UpdateLidar(meas_package.raw_measurements_);
    std::cout<<"NIS_Laser = "<<NIS_ladar_<<"\n";
  }
  else if(use_radar_&&meas_package.sensor_type_== MeasurementPackage::RADAR){
    UpdateRadar(meas_package.raw_measurements_);
    std::cout<<"NIS_Radar = "<<NIS_radar_<<"\n";
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_,2*n_aug_*2+1);
  AugumentSigmaPoints(Xsig_aug);
  SigmaPointPrediction(Xsig_aug,delta_t);
  PredictMeanAndCovariance();
}

void UKF::UpdateLidar(VectorXd &measurement) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  int n_z_ = 2;
  MatrixXd Zsig = MatrixXd::Zero(n_z_,2*n_aug_+1);
  VectorXd Zpred = VectorXd::Zero(n_z_);
  MatrixXd S_ = MatrixXd::Zero(n_z_,n_z_);
  MatrixXd R_ = MatrixXd::Zero(n_z_,n_z_);
  MatrixXd Tc = MatrixXd::Zero(n_x_,n_z_);

  R_(0,0) = std_laspx_*std_laspx_;
  R_(1,1) = std_laspy_*std_laspy_;

  weights_(0) = lambda_/(lambda_+n_aug_);

  for(int i = 1; i<2*n_aug_+1; i++){
    double weight = 0.5/(lambda_+n_aug_);
    weights_(i) = weight;
  }

  for(int i = 0; i<2*n_aug_+1; i++){
    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);
  }

  for(int i = 0;i<2*n_aug_+ 1;i++){
    Zpred = Zpred + (weights_(i)*Zsig.col(i));
  }

  for(int i = 0;i<2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i)- Zpred ; 
    S_ = S_+ (weights_(i)*z_diff*z_diff.transpose());
  }
  S_ = S_ + R_;

  for(int i = 0; i< 2* n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i)-Zpred;
    VectorXd x_diff  =Xsig_pred_.col(i)-x_;
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    Tc = Tc + (weights_(i)* x_diff * z_diff.transpose());
  }
  MatrixXd K = Tc * S_.inverse();
  VectorXd y = measurement - Zpred;

  x_ = x_ + (K * y);
  P_ = P_ - K*S_*K.transpose();

  //NIS evaludation
  NIS_ladar_ = y.transpose()* S_.inverse() * y;

}

void UKF::UpdateRadar(VectorXd &measurement) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  int n_z_ = 3;
  MatrixXd Zsig = MatrixXd::Zero(n_z_,2*n_aug_+1);
  VectorXd Zpred = VectorXd::Zero(n_z_);
  MatrixXd S_ = MatrixXd::Zero(n_z_,n_z_);
  MatrixXd R_ = MatrixXd::Zero(n_z_,n_z_);
  MatrixXd Tc = MatrixXd::Zero(n_x_,n_z_);

  R_(0,0) = std_radr_*std_radr_;
  R_(1,1) = std_radphi_*std_radphi_;
  R_(2,2) = std_radrd_*std_radrd_;

  weights_(0) = lambda_/(lambda_+n_aug_);

  for(int i = 1; i<2*n_aug_+1; i++){
    double weight = 0.5/(lambda_+n_aug_);
    weights_(i) = weight;
  }

  for(int i = 0; i<2*n_aug_+1; i++){
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    Zsig(0,i) = sqrt(p_x*p_x+p_y*p_y);
    Zsig(1,i) = atan2(p_y,p_x);
    if(Zsig(0,i)<0.001){
      Zsig(2,i) = (p_x*v1 + p_y*v2)/0.001;
    }
    else{
      Zsig(2,i) = (p_x*v1 + p_y*v2)/Zsig(0,i);
    }
  }

  for(int i = 0;i<2*n_aug_+ 1;i++){
    Zpred = Zpred + (weights_(i)*Zsig.col(i));
  }

  for(int i = 0;i<2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i)- Zpred ; 
    while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S_ = S_+ (weights_(i)*z_diff*z_diff.transpose());
  }
  S_ = S_ + R_;

  for(int i = 0; i< 2* n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i)-Zpred;
    VectorXd x_diff  =Xsig_pred_.col(i)-x_;
    while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    Tc = Tc + (weights_(i)* x_diff * z_diff.transpose());
  }
  MatrixXd K = Tc * S_.inverse();
  VectorXd y = measurement - Zpred;
  while(y(1)>M_PI) y(1)-=2.*M_PI;
  while(y(1)<-M_PI)y(1)+=2.*M_PI;
  x_ = x_ + (K * y);
  P_ = P_ - K*S_*K.transpose();


 NIS_radar_ = y.transpose()* S_.inverse() * y;


}



void UKF::AugumentSigmaPoints(MatrixXd &Xsig_aug){
    x_aug_.head(n_x_) = x_;

    P_aug_.topLeftCorner(n_x_,n_x_) = P_;

    P_aug_(n_x_,n_x_) = std_a_* std_a_;

    P_aug_(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

    MatrixXd L = P_aug_.llt().matrixL();

    Xsig_aug.col(0) = x_aug_;

    for(int i = 0; i<n_aug_; i++){
      Xsig_aug.col(i+1) = x_aug_ + (sqrt(lambda_+n_aug_)*L.col(i));
      Xsig_aug.col(i+1+n_aug_) = x_aug_ - (sqrt(lambda_+n_aug_)*L.col(i));
    }

}

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double dt){

  for(int i = 0; i<2*n_aug_+1; i++){
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    double px_p, py_p;
    if(fabs(yawd)>0.001){
      px_p = p_x + v/yawd*(sin(yaw+yawd*dt)-sin(yaw));
      py_p = p_y + v/yawd*(- cos(yaw+yawd*dt)+cos(yaw));
    }
    else{
      px_p = p_x + v*cos(yaw)*dt;
      py_p = p_y + v*sin(yaw)*dt;
    }
    double v_p = v;
    double yaw_p = yawd*dt + yaw;
    double yawd_p = yawd;

    px_p = px_p + 0.5* nu_a *dt * dt *cos(yaw);
    py_p = py_p +0.5 *nu_a *dt *dt *sin(yaw);
    v_p = v_p + nu_a * dt;

    yaw_p = yaw_p + 0.5 * nu_yawdd * dt * dt;  
    yawd_p = yawd_p + nu_yawdd * dt;

    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;

  }
  
}

void UKF::PredictMeanAndCovariance(){
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i = 1; i<2*n_aug_+1; i++){
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  x_.fill(0.0);
  for(int i = 0; i< 2* n_aug_+1; i++){
    x_ = x_ + (weights_(i)* Xsig_pred_.col(i));
  }

  P_.fill(0.0);
  for(int i = 0; i<2*n_aug_+1; i++){
    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + (weights_(i)*x_diff*x_diff.transpose());
  }

}

