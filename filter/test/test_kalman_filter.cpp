#include <iostream>

#include "kalman_filter.hpp"

int main(int argc, char const *argv[]) {
  KalmanFilterPtr kf(new KalmanFilter(4, 4, 1));
}
