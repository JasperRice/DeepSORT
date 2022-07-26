#include <iostream>
#include <vector>

#include "kalman_filter.hpp"

void test_kalman_filter() {
  KalmanFilterPtr kf(new KalmanFilter(3, 1, 1));
  float dt = 1.0 / 30.0;
  kf->B << 0.0, 0.0, 0.0;
  kf->F << 1.0, dt, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0;
  kf->H << 1.0, 0.0, 0.0;
  kf->P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
  kf->Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  kf->R << 5.0;

  std::vector<double> measurements = {
      1.04202710058,  1.10726790452,  1.2913511148,    1.48485250951,
      1.72825901034,  1.74216489744,  2.11672039768,   2.14529225112,
      2.16029641405,  2.21269371128,  2.57709350237,   2.6682215744,
      2.51641839428,  2.76034056782,  2.88131780617,   2.88373786518,
      2.9448468727,   2.82866600131,  3.0006601946,    3.12920591669,
      2.858361783,    2.83808170354,  2.68975330958,   2.66533185589,
      2.81613499531,  2.81003612051,  2.88321849354,   2.69789264832,
      2.4342229249,   2.23464791825,  2.30278776224,   2.02069770395,
      1.94393985809,  1.82498398739,  1.52526230354,   1.86967808173,
      1.18073207847,  1.10729605087,  0.916168349913,  0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013,
      -0.602973173813};
  kf->X << measurements[0], 0.0, -9.81;

  MatrixXfr z(1, 1);
  MatrixXfr u(1, 1);

  float t = 0.0;
  for (size_t i = 0; i < measurements.size(); i++) {
    t += dt;
    z << measurements[i];
    u << 0.0;
    kf->predict(u);
    kf->update(z);
    std::cout << "t = " << t << ", "
              << "y[" << i << "] = " << z.transpose() << ", x_hat[" << i
              << "] = " << kf->X.transpose() << std::endl;
  }
}

void test_extended_kalman_filter() {
  ExtendedKalmanFilterPtr ekf(new ExtendedKalmanFilter(5, 1, 1));
}

int main(int argc, char const *argv[]) {
  test_extended_kalman_filter();
  return 0;
}
