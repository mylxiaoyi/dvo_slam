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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include <dvo_slam/keyframe.h>

namespace dvo_slam {

class ORBmatcher {
 public:
  ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat& a, const cv::Mat& b);

  // Search matches between Frame keypoints and projected MapPoints. Returns
  // number of matches
  // Used to track the local map (Tracking)
  //    int SearchByProjection (std::shared_ptr<Frame> F,
  //                            const std::vector<std::shared_ptr<MapPoint>>&
  //                            vpMapPoints,
  //                            const float th = 3);

  // Project MapPoints tracked in last frame into the current frame and search
  // matches.
  // Used to track from previous frame (Tracking)
  int SearchByProjection(KeyframePtr& CurrentFrame,
                         const KeyframePtr& LastFrame, const float th,
                         const bool bMono = false);

  //    // Project MapPoints seen in KeyFrame into the Frame and search matches.
  //    // Used in relocalisation (Tracking)
  //    int SearchByProjection (std::shared_ptr<Frame> CurrentFrame,
  //                            std::shared_ptr<KeyFrame> pKF,
  //                            const std::set<std::shared_ptr<MapPoint>>&
  //                            sAlreadyFound,
  //                            const float th,
  //                            const int ORBdist);

  //    // Project MapPoints using a Similarity Transformation and search
  //    matches.
  //    // Used in loop detection (Loop Closing)
  //    int SearchByProjection (std::shared_ptr<KeyFrame> pKF,
  //                            cv::Mat Scw,
  //                            const std::vector<std::shared_ptr<MapPoint>>&
  //                            vpPoints,
  //                            std::vector<std::shared_ptr<MapPoint>>&
  //                            vpMatched,
  //                            int th);

  //    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  //    // Brute force constrained to ORB that belong to the same vocabulary
  //    node
  //    // (at a certain level)
  //    // Used in Relocalisation and Loop Detection
  //    int SearchByBoW (std::shared_ptr<KeyFrame> pKF,
  //                     std::shared_ptr<Frame> F,
  //                     vector<std::shared_ptr<MapPoint>>& vpMapPointMatches);
  //    int SearchByBoW2 (std::shared_ptr<KeyFrame> pKF,
  //                      std::shared_ptr<KeyFrame> pKF2,
  //                      vector<std::shared_ptr<MapPoint>>& vpMapPointMatches);
  //    int SearchByBoW (std::shared_ptr<KeyFrame> pKF1,
  //                     std::shared_ptr<KeyFrame> pKF2,
  //                     vector<std::shared_ptr<MapPoint>>& vpMatches12);

  //    // Matching for the Map Initialization (only used in the monocular case)
  //    int SearchForInitialization (std::shared_ptr<Frame> F1,
  //                                 std::shared_ptr<Frame> F2,
  //                                 std::vector<cv::Point2f>& vbPrevMatched,
  //                                 std::vector<int>& vnMatches12,
  //                                 int windowSize = 10);

  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  int SearchForTriangulation(
      boost::shared_ptr<Keyframe> pKF1, boost::shared_ptr<Keyframe> pKF2,
      Eigen::Matrix<double, 3, 3> F12,
      std::vector<std::pair<size_t, size_t>>& vMatchedPairs,
      const bool bOnlyStereo);

  //    // Search matches between MapPoints seen in KF1 and KF2 transforming by
  //    a
  //    // Sim3 [s12*R12|t12]
  //    // In the stereo and RGB-D case, s12=1
  //    int SearchBySim3 (std::shared_ptr<KeyFrame> pKF1,
  //                      std::shared_ptr<KeyFrame> pKF2,
  //                      vector<std::shared_ptr<MapPoint>>& vpMatches12,
  //                      const float& s12,
  //                      const cv::Mat& R12,
  //                      const cv::Mat& t12,
  //                      const float th);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  int Fuse(boost::shared_ptr<Keyframe> pKF,
           const vector<boost::shared_ptr<MapPoint>>& vpMapPoints,
           const float th = 3.0);

  //    // Project MapPoints into KeyFrame using a given Sim3 and search for
  //    // duplicated MapPoints.
  //    int Fuse (std::shared_ptr<KeyFrame> pKF,
  //              cv::Mat Scw,
  //              const vector<std::shared_ptr<MapPoint>>& vpPoints,
  //              float th,
  //              vector<std::shared_ptr<MapPoint>>& vpReplacePoint);

 public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

 protected:
  bool CheckDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                             const Eigen::Matrix<double, 3, 3>& F12,
                             const boost::shared_ptr<Keyframe> pKF);

  float RadiusByViewingCos(const float& viewCos);

  void ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1,
                          int& ind2, int& ind3);

  float mfNNratio;
  bool mbCheckOrientation;
};

}  // namespace ORB_SLAM

#endif  // ORBMATCHER_H
