#ifndef DEPPSORT_TRACKER_H
#define DEPPSORT_TRACKER_H

#include <unordered_map>

#include "deepsort_tracking_object.h"

class DeepSortTracker {
 public:
  DeepSortTracker();
  ~DeepSortTracker();
  void Reset();

  void Track(const MatrixXfr& features);
  void Match(const MatrixXfr& detections, const MatrixXfr& features);

  void Initiate(const DetectBox& detection, const MatrixXfr& feature);
  void Predict();
  void Update();

  // Get Properties
  MatrixXfr GetFeature(int id);

  // Input
  void Set();

  // Output
  void GetResult();

 protected:
  int accept;    // After this number of hits, a track is confirmed
  int deadline;  // After this number of missing, a track is lost
  int frame_id;
  int next_id;

  float match_cosine_distance_threshold;
  float match_iou_threshold;
  float object_score_threshold;
  float tentative_frame;

  DeepSortKalmanFilterPtr kf;

  std::unordered_map<int, DeepSortTrackingObjectPtr> vehicles;
};

using DeepSortTrackerPtr = std::shared_ptr<DeepSortTracker>;

#endif