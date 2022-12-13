#include <iostream>

#include "deepsort_data_structure.h"
#include "deepsort_kalman_filter.h"
#include "deepsort_tracking_object.h"
#include "deepsort_tracking_vehicle.h"

int main(int argc, char const *argv[]) {
  DeepSortKalmanFilterPtr kf(new DeepSortKalmanFilter());

  std::cout << "Initial" << std::endl;
  kf->Initiate(255.0, 255.0, 1.0, 10.0);
  MatrixXfr state = kf->GetState();
  MatrixXfr covariance = kf->GetCovariance();
  std::cout << "---- X" << std::endl;
  std::cout << state << std::endl << std::endl;
  std::cout << "---- P" << std::endl;
  std::cout << covariance << std::endl << std::endl;

  std::cout << "Predict" << std::endl;
  kf->Predict();
  state = kf->GetState();
  covariance = kf->GetCovariance();
  std::cout << "---- X" << std::endl;
  std::cout << state << std::endl << std::endl;
  std::cout << "---- P" << std::endl;
  std::cout << covariance << std::endl << std::endl;

  std::cout << "Update" << std::endl;
  MatrixXfr Z(4, 1);
  Z << 256.0, 256.0, 1.0, 11.0;
  kf->Update(Z);
  state = kf->GetState();
  covariance = kf->GetCovariance();
  std::cout << "---- X" << std::endl;
  std::cout << state << std::endl << std::endl;
  std::cout << "---- P" << std::endl;
  std::cout << covariance << std::endl << std::endl;

  std::cout << "Predict" << std::endl;
  kf->Predict();
  state = kf->GetState();
  covariance = kf->GetCovariance();
  std::cout << "---- X" << std::endl;
  std::cout << state << std::endl << std::endl;
  std::cout << "---- P" << std::endl;
  std::cout << covariance << std::endl << std::endl;

  DeepSortTrackingObjectPtr object(new DeepSortTrackingObject());
  DeepSortTrackingVehiclePtr vehicle(new DeepSortTrackingVehicle());
  return 0;
}
