#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <dvo_slam/keyframe.h>
#include <Eigen/Core>
#include <map>

namespace dvo_slam {
class MapPoint {
 public:
  MapPoint();
  MapPoint(Eigen::Vector3d pos, KeyframePtr ref);
  virtual ~MapPoint();

  Eigen::Vector3d GetPos() { return pos_; }
  void SetPos(Eigen::Vector3d pos) { pos_ = pos; }

  cv::Mat GetDescriptor();

  void AddObservation(KeyframePtr keyframe, size_t idx);

  size_t Observations() { return mObservations.size(); }

  void ComputeDistinctiveDescriptors();
  void UpdateNormalAndDepth();

  bool isBad() { return mbBad; }

  bool IsInKeyFrame(KeyframePtr &keyframe) {
    return mObservations.count(keyframe);
  }

  std::map<boost::shared_ptr<Keyframe>, size_t> GetObservations() {
    return mObservations;
  }

  int GetIndexInKeyFrame(boost::shared_ptr<Keyframe> pKF);
  void EraseObservation(boost::shared_ptr<Keyframe> pKF);

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  Eigen::Vector3d GetNormal();
  int PredictScale(const float &currentDist, const float &logScaleFactor);
  void Replace(boost::shared_ptr<MapPoint> pMP);
  void IncreaseVisible(int n=1);
  void IncreaseFound(int n=1);

  // private:
  Eigen::Vector3d pos_;
  KeyframePtr ref_;
  cv::Mat mDescriptor;
  std::map<KeyframePtr, size_t> mObservations;

  bool mbBad;
  long unsigned int mnId;
  static long unsigned int mnNextId;

  int nObs;

  unsigned long int mnBALocalForKF;
  unsigned long int mnFuseCandidateForKF;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Eigen::Vector3d mNormalVector;

  // Tracking counters
  int mnVisible;
  int mnFound;

  boost::shared_ptr<MapPoint> mpReplaced;
};
}

#endif  // MAPPOINT_H
