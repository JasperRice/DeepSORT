#include "deepsort_tracking_object.h"

DeepSortTrackingObject::DeepSortTrackingObject() {}

DeepSortTrackingObject::DeepSortTrackingObject(const MatrixXfr& X,
                                               const MatrixXfr& P,
                                               const MatrixXfr& feature, int id,
                                               float conf, float cls) {
  this->conf = conf;
  this->cls = cls;

  this->accept = 30;
  this->deadline = 30;
  this->dimension = feature.cols();
  this->hit = 1;
  this->id = id;
  this->limit = 60;
  this->missing = 0;

  this->X = X;
  this->P = P;

  this->features = feature;
}

DeepSortTrackingObject::~DeepSortTrackingObject() {}

void DeepSortTrackingObject::Predict(DeepSortKalmanFilterPtr kf) {
  kf->Set(this->X, this->P);
  kf->Predict();
  this->X = kf->GetState();
  this->P = kf->GetCovariance();
}

void DeepSortTrackingObject::Update(DeepSortKalmanFilterPtr kf,
                                    const DetectBox& detection,
                                    const MatrixXfr& feature) {
  float x = (detection.x0 + detection.x1) / 2.0f;
  float y = (detection.y0 + detection.y1) / 2.0f;
  float w = fabsf(detection.x1 - detection.x0);
  float h = fabsf(detection.y1 - detection.y0);
  float a = w / h;

  this->detection = detection;
  this->conf = detection.conf;
  this->cls = detection.cls;

  MatrixXfr Z(4, 1);
  Z << x, y, a, h;
  Update(kf, Z);
  Update(feature);
}

void DeepSortTrackingObject::Update(DeepSortKalmanFilterPtr kf,
                                    const MatrixXfr& Z) {
  kf->Set(this->X, this->P);
  kf->Update(Z);

  this->hit += 1;
  this->missing = 0;
  this->X = kf->GetState();
  this->P = kf->GetCovariance();
}

void DeepSortTrackingObject::Update(const MatrixXfr& feature) {
  if (this->features.rows() < this->limit) {
    this->features.conservativeResize(this->features.rows() + 1,
                                      this->features.cols());
  } else {
    this->features.block(0, 0, this->features.rows() - 1,
                         this->features.cols()) =
        this->features.block(1, 0, this->features.rows() - 1,
                             this->features.cols());
  }
  this->features.row(this->features.rows() - 1) = feature;
}

// Get Properties
int DeepSortTrackingObject::GetClass() { return this->cls; }

int DeepSortTrackingObject::GetID() { return this->id; }

bool DeepSortTrackingObject::isTentative() { return this->hit < this->accept; }

bool DeepSortTrackingObject::isConfirmed() {
  return not(isTentative() or isLost());
}

bool DeepSortTrackingObject::isLost() { return this->missing > this->deadline; }

// Input

// Output
void DeepSortTrackingObject::GetResult() {}