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
class Map;
class KeyframeDatabase;

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

class Keyframe : public boost::enable_shared_from_this<Keyframe> {
 public:
  Keyframe()
      : id_(-1),
        mbFirstConnection(true),
        mbNotErase(false),
        mbToBeErased(false),
        mbBad(false){};
  virtual ~Keyframe(){};

  FI_ATTRIBUTE(Keyframe, unsigned long int, id)
  FI_ATTRIBUTE(Keyframe, dvo::core::RgbdImagePyramid::Ptr, image)
  FI_ATTRIBUTE(Keyframe, Eigen::Affine3d, pose)
  FI_ATTRIBUTE(Keyframe, dvo_slam::TrackingResultEvaluation::ConstPtr,
               evaluation)

  ros::Time timestamp() const { return ros::Time(image()->timestamp()); }

  static bool lId(boost::shared_ptr<Keyframe> pKF1, boost::shared_ptr<Keyframe> pKF2) {
    return pKF1->id() < pKF2->id();
  }

  size_t N;
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;
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

  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;

  Eigen::Matrix<double, 3, 3> mK;

  float mbf, mb;
  float mThDepth;

  std::vector<float> mvuRight, mvDepth;

  // BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // BoW
  //  boost::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  boost::shared_ptr<ORBVocabulary> mpORBvocabulary;

  unsigned long int mnBALocalForKF;
  unsigned long int mnBAFixedForKF;
  unsigned long int mnFuseTargetForKF;

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  long unsigned int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  boost::shared_ptr<Map> mpMap;
  boost::shared_ptr<KeyframeDatabase> mpKeyFrameDB;

  // Variables used by loop closing
  Eigen::Affine3d mTcwGBA;
  Eigen::Affine3d mTcwBefGBA;
  long unsigned int mnBAGlobalForKF;

  void detectFeatures();

  void ComputeStereoFromRGBD(const cv::Mat& imDepth);

  Eigen::Vector2d project(Eigen::Vector3d p);

  Eigen::Vector2d project(double x, double y, double z);

  Eigen::Vector3d unproject(double u, double v, double d);

  Eigen::Vector3d UnprojectStereo(int i);
  Eigen::Vector3d UnprojectStereo(int x, int y);

  bool isInImageBound(Eigen::Vector2d uv);
  bool IsInImage(const float& x, const float& y);

  void AssignFeaturesToGrid();

  bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);

  std::vector<size_t> GetFeaturesInArea(const float& x, const float& y,
                                        const float& r, const int minLevel = -1,
                                        const int maxLevel = -1) const;

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

  std::vector<boost::shared_ptr<Keyframe>> GetVectorCovisibleKeyFrames();
  std::vector<boost::shared_ptr<MapPoint>> GetMapPointMatches();
  void EraseMapPointMatch(const size_t& idx);
  void EraseMapPointMatch(boost::shared_ptr<MapPoint> pMP);

  boost::shared_ptr<MapPoint> GetMapPoint(const size_t& idx);
  void ReplaceMapPointMatch(const size_t& idx, boost::shared_ptr<MapPoint> pMP);

  std::set<boost::shared_ptr<Keyframe>> GetConnectedKeyFrames();

  void UndistortKeyPoints();
  void ComputeImageBounds(const cv::Mat& imLeft);

  Eigen::Vector3d GetCameraCenter();

  void SetBadFlag();
  void EraseConnection(boost::shared_ptr<Keyframe> pKF);
  int GetWeight(boost::shared_ptr<Keyframe> pKF);
  void ChangeParent(boost::shared_ptr<Keyframe> pKF);
  void EraseChild(boost::shared_ptr<Keyframe> pKF);
  void SetNotErase();
  void SetErase();

  set<boost::shared_ptr<MapPoint>> GetMapPoints();

  static bool weightComp(int a, int b) { return a > b; }
  vector<boost::shared_ptr<Keyframe>> GetCovisiblesByWeight(const int& w);
  bool hasChild(boost::shared_ptr<Keyframe> pKF);
  set<boost::shared_ptr<Keyframe>> GetLoopEdges();
  boost::shared_ptr<Keyframe> GetParent();
  void AddLoopEdge(boost::shared_ptr<Keyframe> pKF);
  set<boost::shared_ptr<Keyframe>> GetChilds();
};

typedef boost::shared_ptr<Keyframe> KeyframePtr;
typedef std::vector<KeyframePtr> KeyframeVector;

} /* namespace dvo_slam */
#endif /* KEYFRAME_H_ */
