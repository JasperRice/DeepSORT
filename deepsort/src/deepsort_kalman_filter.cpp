#include "deepsort_kalman_filter.h"

DeepSortKalmanFilter::DeepSortKalmanFilter(bool center) {
  this->center = center;
  if (this->center) {
    this->dimension = 2;
  } else {
    this->dimension = 4;
  }
  this->gating_threshold = chi2inv95[this->dimension];
  this->dt = 1.0;

  std_weight_position = 1.0 / 20.0;
  std_weight_velocity = 1.0 / 160.0;

  this->X = MatrixXfr::Zero(2 * this->dimension, 1);
  this->P = MatrixXfr::Zero(2 * this->dimension, 2 * this->dimension);

  this->F = MatrixXfr::Identity(2 * this->dimension, 2 * this->dimension);
  for (size_t i = 0; i < this->dimension; i++) {
    F(i, this->dimension + i) = this->dt;
  }
  this->H = MatrixXfr::Identity(this->dimension, 2 * this->dimension);
}

DeepSortKalmanFilter::~DeepSortKalmanFilter() {}

void DeepSortKalmanFilter::Initiate(const DetectBox& detection) {
  float x = (detection.x0 + detection.x1) / 2.0;
  float y = (detection.y0 + detection.y1) / 2.0;
  float w = fabs(detection.x0 - detection.x1);
  float h = fabs(detection.y0 - detection.y1);
  float a = fabs(w / h);
  Initiate(x, y, a, h);
}

void DeepSortKalmanFilter::Initiate(float x, float y, float a, float h) {
  if (this->center) {
    this->X << x, y, 0.0, 0.0;
  } else {
    this->X << x, y, a, h, 0.0, 0.0, 0.0, 0.0;
  }

  if (this->center) {
    P.diagonal() << powf(2.0 * this->std_weight_position * h, 2.0),
        powf(2.0 * this->std_weight_position * h, 2.0),
        powf(10.0 * this->std_weight_velocity * h, 2.0),
        powf(10.0 * this->std_weight_velocity * h, 2.0);
  } else {
    P.diagonal() << powf(2.0 * this->std_weight_position * h, 2.0),
        powf(2.0 * this->std_weight_position * h, 2.0), powf(1e-2, 2.0),
        powf(2.0 * this->std_weight_position * h, 2.0),
        powf(10.0 * this->std_weight_velocity * h, 2.0),
        powf(10.0 * this->std_weight_velocity * h, 2.0), powf(10e-5, 2.0),
        powf(10.0 * this->std_weight_velocity * h, 2.0);
  }
}

void DeepSortKalmanFilter::Predict() {
  this->Q = MatrixXfr::Zero(2 * this->dimension, 2 * this->dimension);
  if (this->center) {
    this->Q.diagonal() << powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_velocity * this->X(3), 2.0),
        powf(this->std_weight_velocity * this->X(3), 2.0);
  } else {
    this->Q.diagonal() << powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_position * this->X(3), 2.0), powf(1e-2, 2.0),
        powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_velocity * this->X(3), 2.0),
        powf(this->std_weight_velocity * this->X(3), 2.0), powf(1e-5, 2.0),
        powf(this->std_weight_velocity * this->X(3), 2.0);
  }

  this->X = this->F * this->X;
  this->P = this->F * this->P * (this->F.transpose()) + this->Q;
}

void DeepSortKalmanFilter::Update(const MatrixXfr& Z) {
  this->R = MatrixXfr::Zero(this->dimension, this->dimension);
  if (this->center) {
    this->R.diagonal() << powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_position * this->X(3), 2.0);

    this->Y = Z.block(0, 0, 1, this->dimension) - this->H * this->X;
  } else {
    this->R.diagonal() << powf(this->std_weight_position * this->X(3), 2.0),
        powf(this->std_weight_position * this->X(3), 2.0), powf(1e-1, 2.0),
        powf(this->std_weight_position * this->X(3), 2.0);

    this->Y = Z - this->H * this->X;
  }

  this->S = this->H * this->P * this->H.transpose() + this->R;

  this->K = this->P * this->H.transpose() * this->S.inverse();
  this->X = this->X + this->K * this->Y;
  this->P = this->P - this->K * this->H * this->P;
}

// Input
void DeepSortKalmanFilter::Set(const MatrixXfr& X, const MatrixXfr& P) {
  this->X = X;
  this->P = P;
}

void DeepSortKalmanFilter::SetState(const MatrixXfr& X) { this->X = X; }

void DeepSortKalmanFilter::SetCovariance(const MatrixXfr& P) { this->P = P; }

// Output
MatrixXfr DeepSortKalmanFilter::GetState() { return this->X; }

MatrixXfr DeepSortKalmanFilter::GetCovariance() { return this->P; }