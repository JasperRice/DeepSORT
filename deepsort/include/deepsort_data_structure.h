#ifndef DEEPSORT_DATAT_STRUCTURE_H
#define DEEPSORT_DATAT_STRUCTURE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstddef>
#include <vector>

// >> Sifan
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> MatrixXfr;
typedef std::pair<MatrixXfr, MatrixXfr> MatrixXfrPair;

typedef struct DetectBox {
  DetectBox(float x0 = 0.0f, float y0 = 0.0f, float x1 = 0.0f, float y1 = 0.0f,
            float conf = 0.0f, int cls = -1) {
    this->x0 = x0;
    this->y0 = y0;
    this->x1 = x1;
    this->y1 = y1;
    this->conf = conf;
    this->cls = cls;
  }
  float x0, y0, x1, y1;
  float conf;
  int cls;
} DetectBox;

enum TrackState { Tentative = 1, Confirmed, Lost };
// << Sifan

typedef struct CLSCONF {
  CLSCONF() {
    this->cls = -1;
    this->conf = -1;
  }
  CLSCONF(int cls, float conf) {
    this->cls = cls;
    this->conf = conf;
  }
  int cls;
  float conf;
} CLSCONF;

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;
typedef Eigen::Matrix<float, 1, 256, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 256, Eigen::RowMajor> FEATURESS;

// Result Data Structure
using RESULT_DATA = std::pair<int, DETECTBOX>;

// Tracker Data Structure
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t {
  std::vector<MATCH_DATA> matches;
  std::vector<int> unmatched_tracks;
  std::vector<int> unmatched_detections;
} TRACHER_MATCHD;

#endif  // DEEPSORT_DATAT_STRUCTURE_H