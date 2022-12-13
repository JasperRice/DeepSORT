#ifndef DEEPSORT_KALMAN_FILTER_H
#define DEEPSORT_KALMAN_FILTER_H

#include <math.h>

#include <memory>

#include "deepsort_data_structure.h"

class DeepSortKalmanFilter {
 public:
  const double chi2inv95[10] = {0.0,    3.8415, 5.9915, 7.8147, 9.4877,
                                11.070, 12.592, 14.067, 15.507, 16.919};
  DeepSortKalmanFilter(bool center = false);
  ~DeepSortKalmanFilter();

  void Initiate(const DetectBox& detection);
  void Initiate(float x, float y, float a, float h);
  void Predict();
  void Update(const MatrixXfr& Z);

  // Input
  void Set(const MatrixXfr& X, const MatrixXfr& P);
  void SetState(const MatrixXfr& X);
  void SetCovariance(const MatrixXfr& P);

  // Output
  MatrixXfr GetState();
  MatrixXfr GetCovariance();

 protected:
  bool center;
  int dimension;
  float gating_threshold;
  float dt;

  float std_weight_position;
  float std_weight_velocity;

  MatrixXfr X;
  MatrixXfr P;

  MatrixXfr F;
  MatrixXfr H;

  MatrixXfr Q;
  MatrixXfr R;

  MatrixXfr Y;
  MatrixXfr K;
  MatrixXfr S;
};

using DeepSortKalmanFilterPtr = std::shared_ptr<DeepSortKalmanFilter>;

#endif  // DEEPSORT_KALMAN_FILTER_H
