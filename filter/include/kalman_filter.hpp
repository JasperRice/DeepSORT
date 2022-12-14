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
  const float sigma_x = 3.12152113e-01;
  const float sigma_y = 5.72281037e-02;
  const float sigma_vx = 6.29019664e+00;
  const float sigma_vy = 1.41461263e+00;
  const float sigma_ax = 1.49371453e+02;
  const float sigma_ay = 3.81161792e+01;

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