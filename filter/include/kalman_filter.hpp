#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> MatrixXfr;

class KalmanFilter {
 protected:
  int dim_x;
  int dim_z;
  int dim_u;

 public:
  MatrixXfr B;  // Control Matrix
  MatrixXfr F;  // Predict Matrix
  MatrixXfr H;  // Observation Matrix
  MatrixXfr I;  // Identity Matrix
  MatrixXfr K;  // Kalman Gain
  MatrixXfr P;  // Covariance
  MatrixXfr Q;  // Process Noise
  MatrixXfr R;  // Measurement Noise
  MatrixXfr X;  // State

 public:
  KalmanFilter(int dim_x, int dim_z, int dim_u = 0);
  ~KalmanFilter();
  virtual void init();
  virtual void predict();
  virtual void predict(const MatrixXfr &u);
  virtual void update(const MatrixXfr &z);
};

using KalmanFilterPtr = std::shared_ptr<KalmanFilter>;

#endif