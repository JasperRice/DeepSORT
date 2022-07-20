#include "deepsort_object_merge.h"

DeepSortObjectMerge::DeepSortObjectMerge() {}

DeepSortObjectMerge::~DeepSortObjectMerge() {}

void DeepSortObjectMerge::Reset() { this->merged.clear(); }

void DeepSortObjectMerge::Merge(
    std::vector<DeepSortTrackingObjectPtr>& objects) {
  /* Add new objects from objects to merged */
  if (this->merged.size() == 0) {
    
  }
}