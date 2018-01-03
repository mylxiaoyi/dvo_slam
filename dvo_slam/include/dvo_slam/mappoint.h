/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <mutex>
#include <map>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace dvo_slam {

class Keyframe;
class Map;

class MapPoint : public boost::enable_shared_from_this<MapPoint> {
 public:
  MapPoint(const Eigen::Vector3d& Pos, boost::shared_ptr<Keyframe> pRefKF, boost::shared_ptr<Map> pMap);
//  MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame, const int& idxF);

  void SetWorldPos(const Eigen::Vector3d& Pos);
  Eigen::Vector3d GetWorldPos();

  Eigen::Vector3d GetNormal();
  boost::shared_ptr<Keyframe> GetReferenceKeyFrame();

  std::map<boost::shared_ptr<Keyframe>, size_t> GetObservations();
  int Observations();

  void AddObservation(boost::shared_ptr<Keyframe> pKF, size_t idx);
  void EraseObservation(boost::shared_ptr<Keyframe> pKF);

  int GetIndexInKeyFrame(boost::shared_ptr<Keyframe> pKF);
  bool IsInKeyFrame(boost::shared_ptr<Keyframe> pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(boost::shared_ptr<MapPoint> pMP);
  boost::shared_ptr<MapPoint> GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  int PredictScale(const float& currentDist, boost::shared_ptr<Keyframe> pKF);
//  int PredictScale(const float& currentDist, Frame* pF);

 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  Eigen::Vector3d mPosGBA;
  long unsigned int mnBAGlobalForKF;

  static std::mutex mGlobalMutex;

 protected:
  // Position in absolute coordinates
  Eigen::Vector3d mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<boost::shared_ptr<Keyframe>, size_t> mObservations;

  // Mean viewing direction
  Eigen::Vector3d mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  boost::shared_ptr<Keyframe> mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  boost::shared_ptr<MapPoint> mpReplaced;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  boost::shared_ptr<Map> mpMap;

  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
};

}  // namespace ORB_SLAM

#endif  // MAPPOINT_H
