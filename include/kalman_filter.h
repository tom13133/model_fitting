#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <ros/ros.h>

class KalmanFilter
{
 public:
  KalmanFilter();

  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param u_in Initial control-input
   * @param P Initial state covariance
   * @param F Transition matrix
   * @param B Control-input matrix
   * @param Q Process covariance matrix
   * @param H Measurement matrix
   * @param R Measurement covariance matrix
   */
  void Init(const Eigen::VectorXd &x_in,
            const Eigen::VectorXd &u_in,
            const Eigen::MatrixXd &P,
            const Eigen::MatrixXd &F,
            const Eigen::MatrixXd &B,
            const Eigen::MatrixXd &Q,
            const Eigen::MatrixXd &H,
            const Eigen::MatrixXd &R,
            const double t); 

  void Predict();

  // @param z measurement
  void Update(const Eigen::VectorXd &z);

  void set_x(const Eigen::VectorXd &x_in);

  void set_t(const double t_new);

  void set_F(const Eigen::MatrixXd &F);

  Eigen::VectorXd get_x();
  double get_t();

 private:
  // state vector
  Eigen::VectorXd x_;
  // control input vector
  Eigen::VectorXd u_;  
  // state covariance matrix
  Eigen::MatrixXd P_;
  // state transistion matrix
  Eigen::MatrixXd F_;
  // control input matrix
  Eigen::MatrixXd B_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;
  // estimate time stamp
  double t_;

};

#endif 