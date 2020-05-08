#include <math.h>
#include "kalman_filter.h"


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(const Eigen::VectorXd &x_in,
                        const Eigen::VectorXd &u_in,
                        const Eigen::MatrixXd &P,
                        const Eigen::MatrixXd &F,
                        const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q,
                        const Eigen::MatrixXd &H,
                        const Eigen::MatrixXd &R,
                        const double t) {
  x_ = x_in;
  u_ = u_in;
  P_ = P;
  F_ = F;
  B_ = B;
  Q_ = Q;
  H_ = H;
  R_ = R;
  t_ = t;
}


void KalmanFilter::Predict() {
  Eigen::MatrixXd Ft = F_.transpose();
  x_ = F_ * x_ + B_ * u_;
  P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::VectorXd z_pred = H_ * x_;
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}


void KalmanFilter::set_x(const Eigen::VectorXd &x_in) {
  x_ = x_in;
}


void KalmanFilter::set_t(const double t_new) {
  double dT = t_new - t_;
  F_(0, 2) = dT;
  F_(1, 3) = dT;
  t_ = t_new;
}

void KalmanFilter::set_F(const Eigen::MatrixXd &F) {
  F_ = F;
}


Eigen::VectorXd KalmanFilter::get_x() {
  return x_;
}

double KalmanFilter::get_t() {
  return t_;
}