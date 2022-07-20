#ifndef DEEPSORT_TRACKING_VEHICLE_H
#define DEEPSORT_TRACKING_VEHICLE_H

#include "deepsort_tracking_object.h"

class DeepSortTrackingPedestrian : public DeepSortTrackingObject {
 public:
  DeepSortTrackingPedestrian(/* args */);
  ~DeepSortTrackingPedestrian();

 protected:
};

using DeepSortTrackingPedestrianPtr =
    std::shared_ptr<DeepSortTrackingPedestrian>;

#endif