#include "deepsort_tracker.h"

DeepSortTracker::DeepSortTracker() { Reset(); }

DeepSortTracker::~DeepSortTracker() {}

void DeepSortTracker::Reset() {
  this->frame_id = 0;
  this->next_id = 0;
  this->vehicles.clear();
}

void DeepSortTracker::Match(const MatrixXfr& detections,
                            const MatrixXfr& features) {}

void DeepSortTracker::Initiate(const DetectBox& detection,
                               const MatrixXfr& feature) {
  // Initialize a track.
  kf->Initiate(detection);
  MatrixXfr X = kf->GetState();
  MatrixXfr P = kf->GetCovariance();

  this->vehicles.emplace(
      this->next_id,
      DeepSortTrackingObjectPtr(new DeepSortTrackingObject(
          X, P, feature, this->next_id, detection.conf, detection.cls)));
  this->next_id += 1;
}

// Input
void DeepSortTracker::Set() {}

// Output
void DeepSortTracker::GetResult() {}