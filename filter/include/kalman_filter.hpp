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
  MatrixXfr B;  // Control transition matrix
  MatrixXfr F;  // State transition matrix
  MatrixXfr H;  // Measurement function
  MatrixXfr I;  // Identity matrix
  MatrixXfr K;  // Kalman gain of the update step
  MatrixXfr P;  // Covariance matrix
  MatrixXfr Q;  // Process noise matrix
  MatrixXfr R;  // Measurement noise matrix
  MatrixXfr S;  // Systen uncertaintly projected to measurement space
  MatrixXfr X;  // State estimate vector

 public:
  KalmanFilter(int dim_x, int dim_z, int dim_u = 0);
  ~KalmanFilter();
  virtual void init();
  virtual void predict();
  virtual void predict(const MatrixXfr &u);
  virtual void update(const MatrixXfr &z);
};

class ExtendedKalmanFilter : public KalmanFilter {
 public:
  MatrixXfr hx;

 public:
  ExtendedKalmanFilter(int dim_x, int dim_z, int dim_u = 0);
  ~ExtendedKalmanFilter();
  virtual void update(const MatrixXfr &z,
                      const MatrixXfr (*HJacobian)(MatrixXfr),
                      const MatrixXfr (*Hx)(MatrixXfr));
};

using KalmanFilterPtr = std::shared_ptr<KalmanFilter>;
using ExtendedKalmanFilterPtr = std::shared_ptr<ExtendedKalmanFilter>;

#endif