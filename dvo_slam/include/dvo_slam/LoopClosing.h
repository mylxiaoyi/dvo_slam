/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/RSLAM>
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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <boost/shared_ptr.hpp>
#include <map>
#include <mutex>
#include <thread>

#include "dvo_slam/orbvocabulary.h"

#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace dvo_slam {

class Keyframe;
class Map;
class MapPoint;
class KeyframeDatabase;

class LoopClosing {
 public:
  typedef std::pair<set<boost::shared_ptr<Keyframe>>, int> ConsistentGroup;
  typedef std::map<boost::shared_ptr<Keyframe>, g2o::Sim3,
                   std::less<boost::shared_ptr<Keyframe>>,
                   Eigen::aligned_allocator<
                       std::pair<const boost::shared_ptr<Keyframe>, g2o::Sim3>>>
      KeyFrameAndPose;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopClosing(boost::shared_ptr<Map> pMap,
              boost::shared_ptr<KeyframeDatabase> pDB,
              boost::shared_ptr<ORBVocabulary> pVoc, const bool bFixScale);
  virtual ~LoopClosing();

  void execLoop();

//  void SetTracker(boost::shared_ptr<Tracking> pTracker);

//  void SetLocalMapper(boost::shared_ptr<LocalMapping> pLocalMapper);

  // Main function
  //    void Run();

  void InsertKeyFrame(boost::shared_ptr<Keyframe> pKF);

  void RequestReset();

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(unsigned long nLoopKF);

  bool isRunningGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
  }
  bool isFinishedGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
  }

  void RequestFinish();

  bool isFinished();

  bool loop_thread_shutdown_;

 protected:
  virtual void run();

  bool CheckNewKeyFrames();

  bool DetectLoop();

  bool ComputeSim3();

  void SearchAndFuse(const KeyFrameAndPose& CorrectedPosesMap);

  void CorrectLoop();

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  boost::shared_ptr<Map> mpMap;
  //  boost::shared_ptr<Tracking> mpTracker;

  boost::shared_ptr<KeyframeDatabase> mpKeyFrameDB;
  boost::shared_ptr<ORBVocabulary> mpORBVocabulary;

  //  boost::shared_ptr<LocalMapping> mpLocalMapper;

  std::list<boost::shared_ptr<Keyframe>> mlpLoopKeyFrameQueue;

  std::mutex mMutexLoopQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  boost::shared_ptr<Keyframe> mpCurrentKF;
  boost::shared_ptr<Keyframe> mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<boost::shared_ptr<Keyframe>> mvpEnoughConsistentCandidates;
  std::vector<boost::shared_ptr<Keyframe>> mvpCurrentConnectedKFs;
  std::vector<boost::shared_ptr<MapPoint>> mvpCurrentMatchedPoints;
  std::vector<boost::shared_ptr<MapPoint>> mvpLoopMapPoints;
  cv::Mat mScw;
  g2o::Sim3 mg2oScw;

  long unsigned int mLastLoopKFid;

  // Variables related to Global Bundle Adjustment
  bool mbRunningGBA;
  bool mbFinishedGBA;
  bool mbStopGBA;
  std::mutex mMutexGBA;
  std::thread* mpThreadGBA;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;
};

}  // namespace ORB_SLAM

#endif  // LOOPCLOSING_H
