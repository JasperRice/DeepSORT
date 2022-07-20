#ifndef DEEPSORT_TRACKING_VEHICLE_H
#define DEEPSORT_TRACKING_VEHICLE_H

#include <memory>

#include "deepsort_tracking_object.h"

class DeepSortTrackingVehicle : public DeepSortTrackingObject {
 public:
  DeepSortTrackingVehicle();
  ~DeepSortTrackingVehicle();

 protected:
};

using DeepSortTrackingVehiclePtr = std::shared_ptr<DeepSortTrackingVehicle>;

#endif