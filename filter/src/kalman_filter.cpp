#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(int dim_x, int dim_z, int dim_u) {
  this->dim_x = dim_x;
  this->dim_z = dim_z;
  this->dim_u = dim_u;
  init();
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init() {
  if (dim_u > 0) {
    B = MatrixXfr::Identity(dim_x, dim_u);
  }
  F = MatrixXfr::Identity(dim_x, dim_x);
  H = MatrixXfr::Identity(dim_z, dim_x);
  I = MatrixXfr::Identity(dim_x, dim_x);
  K = MatrixXfr::Zero(dim_x, dim_z);
  P = MatrixXfr::Identity(dim_x, dim_x);
  Q = MatrixXfr::Identity(dim_x, dim_x);
  R = MatrixXfr::Identity(dim_z, dim_z);
  X = MatrixXfr::Zero(dim_x, 1);
}

void KalmanFilter::predict() {
  X = F * X;
  P = F * P * F.transpose() + Q;
}

void KalmanFilter::predict(const MatrixXfr &u) {
  predict();
  if (dim_u > 0) {
    X += B * u;
  }
}

void KalmanFilter::update(const MatrixXfr &z) {
  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();
  X = X + K * (z - H * X);
  P = (I - K * H) * P;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(int dim_x, int dim_z, int dim_u)
    : KalmanFilter(dim_x, dim_z, dim_u) {}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::update(const MatrixXfr &z,
                                  const MatrixXfr (*HJacobian)(MatrixXfr),
                                  const MatrixXfr (*Hx)(MatrixXfr)) {
  H = HJacobian(X);
  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();

  hx = Hx(X);
  X = X + K * hx;
  P = (I - K * H) * P;
}