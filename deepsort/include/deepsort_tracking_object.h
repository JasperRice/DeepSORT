#ifndef DEEPSORT_TRACKING_OBJECT_H
#define DEEPSORT_TRACKING_OBJECT_H

#include <memory>
#include <vector>

#include "deepsort_data_structure.h"
#include "deepsort_kalman_filter.h"

class DeepSortTrackingObject {
 public:
  DeepSortTrackingObject();
  DeepSortTrackingObject(const MatrixXfr& X, const MatrixXfr& P,
                         const MatrixXfr& feature, int id, float conf,
                         float cls);
  ~DeepSortTrackingObject();

  void Predict(DeepSortKalmanFilterPtr kf);
  void Update(DeepSortKalmanFilterPtr kf, const DetectBox& detection,
              const MatrixXfr& feature);
  void Update(DeepSortKalmanFilterPtr kf, const MatrixXfr& Z);
  void Update(const MatrixXfr& feature);

  // Get Properties
  int GetClass();
  int GetID();
  bool isTentative();
  bool isConfirmed();
  bool isLost();

  // Input

  // Output
  void GetResult();

 protected:
  // Perception Info
  DetectBox detection;  // Original detection result
  float conf;
  float cls;

  // Tracking Info
  int accept;     // After this number of hits, a track is confirmed
  int deadline;   // After this number of missing, a track is lost
  int dimension;  // Dimension of the feature
  int hit;        // Count the number of match
  int id;         // The id of the track
  int limit;      // The maximum number of features that would be saved
  int missing;    // Count the number of disappear
  MatrixXfr X;
  MatrixXfr P;
  MatrixXfr features;

  // Attribute Info
  
};

using DeepSortTrackingObjectPtr = std::shared_ptr<DeepSortTrackingObject>;

#endif