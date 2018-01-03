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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "dvo_slam/LoopClosing.h"

namespace dvo_slam {

// class LoopClosing;
class Keyframe;
class Map;
class MapPoint;

class Optimizer {
 public:
  void static BundleAdjustment(const vector<boost::shared_ptr<Keyframe>>& vpKF,
                               const vector<boost::shared_ptr<MapPoint>>& vpMP,
                               int nIterations = 5, bool* pbStopFlag = NULL,
                               const unsigned long nLoopKF = 0,
                               const bool bRobust = true);
  void static GlobalBundleAdjustemnt(boost::shared_ptr<Map> pMap,
                                     int nIterations = 5,
                                     bool* pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
  void static LocalBundleAdjustment(boost::shared_ptr<Keyframe> pKF,
                                    bool* pbStopFlag,
                                    boost::shared_ptr<Map> pMap);
  int static PoseOptimization(boost::shared_ptr<Keyframe> pFrame);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  void static OptimizeEssentialGraph(
      boost::shared_ptr<Map> pMap, boost::shared_ptr<Keyframe> pLoopKF,
      boost::shared_ptr<Keyframe> pCurKF,
      const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose& CorrectedSim3,
      const std::map<boost::shared_ptr<Keyframe>,
                     std::set<boost::shared_ptr<Keyframe>>>& LoopConnections,
      const bool& bFixScale);

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
  static int OptimizeSim3(boost::shared_ptr<Keyframe> pKF1,
                          boost::shared_ptr<Keyframe> pKF2,
                          std::vector<boost::shared_ptr<MapPoint>>& vpMatches1,
                          g2o::Sim3& g2oS12, const float th2,
                          const bool bFixScale);

  //    static int OptimizeSim3FitGPS(std::vector<boost::shared_ptr<KeyFrame> >
  //    &vpKFs,
  //                                   std::vector<cv::Point3d> &vGPSPoints,
  //                                   g2o::Sim3& g2oSim3, const float th2);
};

}  // namespace ORB_SLAM

#endif  // OPTIMIZER_H
