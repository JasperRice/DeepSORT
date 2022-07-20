#ifndef DEEPSORT_OBJECT_MERGE_H
#define DEEPSORT_OBJECT_MERGE_H

#include "deepsort_tracker.h"

class DeepSortObjectMerge {
 public:
  DeepSortObjectMerge();
  ~DeepSortObjectMerge();
  
  void Reset();
  void Merge(std::vector<DeepSortTrackingObjectPtr>& objects);

 protected:
  std::vector<DeepSortTrackingObjectPtr>& merged;
};

#endif