/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical
 * University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <dvo/core/rgbd_image.h>
#include <dvo/util/fluent_interface.h>

#include <dvo_slam/tracking_result_evaluation.h>

#include <ros/time.h>
#include <Eigen/Geometry>
#include <vector>

#include <dvo_slam/ORBextractor.h>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace dvo_slam {

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

class Keyframe : public boost::enable_shared_from_this<Keyframe> {
 public:
  Keyframe() : id_(-1), mbBad(false), mbFirstConnection(true){};
  virtual ~Keyframe(){};

  FI_ATTRIBUTE(Keyframe, unsigned long int, id)
  FI_ATTRIBUTE(Keyframe, dvo::core::RgbdImagePyramid::Ptr, image)
  FI_ATTRIBUTE(Keyframe, Eigen::Affine3d, pose)
  FI_ATTRIBUTE(Keyframe, dvo_slam::TrackingResultEvaluation::ConstPtr,
               evaluation)

  ros::Time timestamp() const { return ros::Time(image()->timestamp()); }

  size_t N;
  std::vector<cv::KeyPoint> mvKeys;
  cv::Mat mDescriptors;
  std::vector<boost::shared_ptr<MapPoint>> mvpMapPoints;
  std::vector<bool> mvbOutlier;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvInvScaleFactors;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;
  static bool mbInitialComputations;

  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  std::map<boost::shared_ptr<Keyframe>, int> mConnectedKeyFrameWeights;
  std::vector<boost::shared_ptr<Keyframe>> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;
  std::vector<long unsigned int> mvOrderedConnectedKeyFrames;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  boost::shared_ptr<Keyframe> mpParent;
  std::set<boost::shared_ptr<Keyframe>> mspChildrens;
  std::set<boost::shared_ptr<Keyframe>> mspLoopEdges;

  bool mbBad;

  float fx, fy;
  float cx, cy;
  float mbf, mb;

  std::vector<float> mvuRight, mvDepth;

  // BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // BoW
  //  boost::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  boost::shared_ptr<ORBVocabulary> mpORBvocabulary;

  unsigned long int mnBALocalForKF;
  unsigned long int mnBAFixedForKF;

  void detectFeatures();

  void ComputeStereoFromRGBD(const cv::Mat& imDepth);

  Eigen::Vector2d project(Eigen::Vector3d p);

  Eigen::Vector2d project(double x, double y, double z);

  Eigen::Vector3d unproject(double u, double v, double d);

  Eigen::Vector3d UnprojectStereo (int i);

  bool isInImageBound(Eigen::Vector2d uv);

  void AssignFeaturesToGrid();

  bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);

  std::vector<size_t> GetFeaturesInArea(const float& x, const float& y,
                                        const float& r, const int minLevel = -1,
                                        const int maxLevel = -1) const;

  void CreateInitMap();

  void AddMapPoint(boost::shared_ptr<MapPoint> pMP, size_t idx);

  bool isBad() { return mbBad; }

  void UpdateConnections();

  void AddConnection(boost::shared_ptr<Keyframe> pKF, const int& weight);

  void UpdateBestCovisibles();

  // Spanning tree functions
  void AddChild(boost::shared_ptr<Keyframe> pKF);
  //  void EraseChild(std::shared_ptr<KeyFrame> pKF);
  //  void ChangeParent(std::shared_ptr<KeyFrame> pKF);
  //  set<std::shared_ptr<KeyFrame> > GetChilds();
  //  std::shared_ptr<KeyFrame> GetParent();
  //  bool hasChild(std::shared_ptr<KeyFrame> pKF);

  std::vector<boost::shared_ptr<Keyframe>> GetBestCovisibilityKeyFrames(
      const int& N);

  void ComputeBoW();
  std::vector<cv::Mat> toDescriptorVector(const cv::Mat& descriptor);

  std::vector<boost::shared_ptr<Keyframe>> GetVectorCovisibleKeyFrames ();
  std::vector<boost::shared_ptr<MapPoint>> GetMapPointMatches ();
  void EraseMapPointMatch(const size_t &idx);
  void EraseMapPointMatch(boost::shared_ptr<MapPoint> pMP);

  boost::shared_ptr<MapPoint> GetMapPoint(const size_t &idx);
};

typedef boost::shared_ptr<Keyframe> KeyframePtr;
typedef std::vector<KeyframePtr> KeyframeVector;

} /* namespace dvo_slam */
#endif /* KEYFRAME_H_ */
