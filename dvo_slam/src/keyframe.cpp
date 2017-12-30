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

#include <dvo_slam/keyframe.h>
#include <dvo_slam/mappoint.h>
#include <dvo_slam/Map.h>
#include <dvo_slam/KeyFrameDatabase.h>

namespace dvo_slam {

float Keyframe::fx = 0.0;
float Keyframe::fy = 0.0;
float Keyframe::cx = 0.0;
float Keyframe::cy = 0.0;
float Keyframe::invfx = 0.0;
float Keyframe::invfy = 0.0;
float Keyframe::mnMinX = 0.0;
float Keyframe::mnMaxX = 0.0;
float Keyframe::mnMinY = 0.0;
float Keyframe::mnMaxY = 0.0;
bool Keyframe::mbInitialComputations = true;

float Keyframe::mfGridElementHeightInv = 1.0;
float Keyframe::mfGridElementWidthInv = 1.0;

void Keyframe::detectFeatures() {
  ORBextractor extractor(1000, 1.2, 8, 20, 7);
  //  cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
  cv::Mat img;
  image()->level(0).intensity.convertTo(img, CV_8UC1);
  extractor(img, cv::Mat(), mvKeys, mDescriptors);
  //  orb->detectAndCompute(img, cv::Mat(), mvKeys, mDescriptors);
  //  orb->detect(img, mvKeys);
  //  orb->compute(img, mvKeys, mDescriptors);

  N = mvKeys.size();
  if (mvKeys.empty()) return;

  // Scale Level Info
  mnScaleLevels = extractor.GetLevels();
  mfScaleFactor = extractor.GetScaleFactor();
  mfLogScaleFactor = std::log(mfScaleFactor);
  mvScaleFactors = extractor.GetScaleFactors();
  mvInvScaleFactors = extractor.GetInverseScaleFactors();
  mvLevelSigma2 = extractor.GetScaleSigmaSquares();
  mvInvLevelSigma2 = extractor.GetInverseScaleSigmaSquares();

  UndistortKeyPoints();

  mbf = 40.0;
  cv::Mat& depth = image()->level(0).depth;
  ComputeStereoFromRGBD(depth);

  mvpMapPoints = std::vector<boost::shared_ptr<MapPoint>>(N, nullptr);
  mvbOutlier = std::vector<bool>(N, false);

  if (mbInitialComputations) {
    ComputeImageBounds(img);

    mfGridElementWidthInv =
        static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv =
        static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    const dvo::core::RgbdCamera& camera = image()->level(0).camera();
    fx = camera.intrinsics().fx();
    fy = camera.intrinsics().fy();
    cx = camera.intrinsics().ox();
    cy = camera.intrinsics().oy();
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  mb = mbf / fx;

  mThDepth = mbf * 40.0 / fx;

  AssignFeaturesToGrid();
}

void Keyframe::ComputeStereoFromRGBD(const cv::Mat& imDepth) {
  mvuRight = std::vector<float>(N, -1);
  mvDepth = std::vector<float>(N, -1);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint& kp = mvKeys[i];
    const cv::KeyPoint& kpU = mvKeysUn[i];

    const float& v = kp.pt.y;
    const float& u = kp.pt.x;

    const float d = imDepth.at<float>(v, u);

    if (d > 0) {
      mvDepth[i] = d;
      mvuRight[i] = kpU.pt.x - mbf / d;
      //      std::cout << "d = " << d << ", x = " << kpU.pt.x
      //                << ", right = " << mvuRight[i] << ", mbf = " << mbf
      //                << std::endl;
    }
  }
}

Eigen::Vector2d Keyframe::project(Eigen::Vector3d p) {
  return project(p[0], p[1], p[2]);
}

Eigen::Vector2d Keyframe::project(double x, double y, double z) {
  double u = fx * x / z + cx;
  double v = fy * y / z + cy;
  return Eigen::Vector2d(u, v);
}

Eigen::Vector3d Keyframe::unproject(double u, double v, double d) {
  double x = (u - cx) / fx;
  double y = (v - cy) / fy;
  return Eigen::Vector3d(x, y, 1) * d;
}

Eigen::Vector3d Keyframe::UnprojectStereo(int i) {
  const float d = mvDepth[i];
  if (d > 0) {
    Eigen::Vector3d x3Dc = unproject(mvKeysUn[i].pt.x, mvKeysUn[i].pt.y, d);
    return pose() * x3Dc;
  } else
    return Eigen::Vector3d(0, 0, 0);
}

Eigen::Vector3d Keyframe::UnprojectStereo(int x, int y) {
  cv::Mat depth = image()->level(0).depth;
  float d = depth.at<float>(y, x);
  if (d > 0) {
    Eigen::Vector3d x3Dc = unproject(x, y, d);
    return pose() * x3Dc;
  } else
    return Eigen::Vector3d(0, 0, 0);
}

bool Keyframe::isInImageBound(Eigen::Vector2d uv) {
  const dvo::core::RgbdCamera& camera = image()->camera_.level(0);
  int width = camera.width();
  int height = camera.height();
  return (uv[0] >= 0 && uv[0] < width && uv[1] >= 0 && uv[1] < height);
}

bool Keyframe::IsInImage(const float& x, const float& y) {
  const dvo::core::RgbdCamera& camera = image()->camera_.level(0);
  int width = camera.width();
  int height = camera.height();
  return (x >= 0 && x < width && y >= 0 && y < height);
}

bool Keyframe::PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY) {
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of
  // the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 ||
      posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}

void Keyframe::AssignFeaturesToGrid() {
  int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
      mGrid[i][j].reserve(nReserve);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint& kp = mvKeysUn[i];

    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY))
      mGrid[nGridPosX][nGridPosY].push_back(i);
  }
}

std::vector<size_t> Keyframe::GetFeaturesInArea(const float& x, const float& y,
                                                const float& r,
                                                const int minLevel,
                                                const int maxLevel) const {
  std::vector<size_t> vIndices;
  vIndices.reserve(N);

  const int nMinCellX =
      std::max(0, (int)std::floor((x - mnMinX - r) * mfGridElementWidthInv));
  if (nMinCellX >= FRAME_GRID_COLS) return vIndices;

  const int nMaxCellX =
      std::min((int)FRAME_GRID_COLS - 1,
               (int)std::ceil((x - mnMinX + r) * mfGridElementWidthInv));
  if (nMaxCellX < 0) return vIndices;

  const int nMinCellY =
      std::max(0, (int)std::floor((y - mnMinY - r) * mfGridElementHeightInv));
  if (nMinCellY >= FRAME_GRID_ROWS) return vIndices;

  const int nMaxCellY =
      std::min((int)FRAME_GRID_ROWS - 1,
               (int)std::ceil((y - mnMinY + r) * mfGridElementHeightInv));
  if (nMaxCellY < 0) return vIndices;

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<size_t> vCell = mGrid[ix][iy];
      if (vCell.empty()) continue;

      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        //        const cv::KeyPoint& kpUn = mvKeysUn[vCell[j]];
        const cv::KeyPoint& kpUn = mvKeys[vCell[j]];
        if (bCheckLevels) {
          if (kpUn.octave < minLevel) continue;
          if (maxLevel >= 0)
            if (kpUn.octave > maxLevel) continue;
        }

        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (std::fabs(distx) < r && std::fabs(disty) < r)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

void Keyframe::AddMapPoint(boost::shared_ptr<MapPoint> pMP, size_t idx) {
  mvpMapPoints[idx] = pMP;
}

void Keyframe::UpdateConnections() {
  std::map<boost::shared_ptr<Keyframe>, int> KFcounter;

  std::vector<boost::shared_ptr<MapPoint>> vpMP;

  {
    //    unique_lock<mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  // For all map points in keyframe check in which other keyframes are they
  // seen
  // Increase counter for those keyframes
  for (std::vector<boost::shared_ptr<MapPoint>>::iterator vit = vpMP.begin(),
                                                          vend = vpMP.end();
       vit != vend; vit++) {
    boost::shared_ptr<MapPoint> pMP = *vit;

    if (!pMP) continue;

    if (pMP->isBad()) continue;

    std::map<boost::shared_ptr<Keyframe>, size_t> observations =
        pMP->GetObservations();

    for (std::map<boost::shared_ptr<Keyframe>, size_t>::iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      if (mit->first->id() == id()) continue;
      KFcounter[mit->first]++;
    }
  }

  // This should not happen
  if (KFcounter.empty()) return;

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum
  // counter
  int nmax = 0;
  boost::shared_ptr<Keyframe> pKFmax = NULL;
  int th = 15;

  std::vector<std::pair<int, boost::shared_ptr<Keyframe>>> vPairs;
  vPairs.reserve(KFcounter.size());
  for (std::map<boost::shared_ptr<Keyframe>, int>::iterator
           mit = KFcounter.begin(),
           mend = KFcounter.end();
       mit != mend; mit++) {
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }
    if (mit->second >= th) {
      vPairs.push_back(
          std::pair<int, boost::shared_ptr<Keyframe>>(mit->second, mit->first));
      (mit->first)->AddConnection(shared_from_this(), mit->second);
    }
  }

  if (vPairs.empty()) {
    vPairs.push_back(std::pair<int, boost::shared_ptr<Keyframe>>(nmax, pKFmax));
    pKFmax->AddConnection(shared_from_this(), nmax);
  }

  std::sort(vPairs.begin(), vPairs.end());
  std::list<boost::shared_ptr<Keyframe>> lKFs;
  std::list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  {
    //    unique_lock<mutex> lockCon(mMutexConnections);

    // mspConnectedKeyFrames = spConnectedKeyFrames;
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames =
        std::vector<boost::shared_ptr<Keyframe>>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && id() != 1) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(shared_from_this());
      mbFirstConnection = false;
    }
  }
}

void Keyframe::AddConnection(boost::shared_ptr<Keyframe> pKF,
                             const int& weight) {
  {
    //        unique_lock<mutex> lock (mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void Keyframe::UpdateBestCovisibles() {
  //    unique_lock<mutex> lock (mMutexConnections);
  std::vector<std::pair<int, boost::shared_ptr<Keyframe>>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (std::map<boost::shared_ptr<Keyframe>, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    vPairs.push_back(std::make_pair(mit->second, mit->first));

  std::sort(vPairs.begin(), vPairs.end());
  std::list<boost::shared_ptr<Keyframe>> lKFs;
  std::list<int> lWs;
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames =
      std::vector<boost::shared_ptr<Keyframe>>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

void Keyframe::AddChild(boost::shared_ptr<Keyframe> pKF) {
  mspChildrens.insert(pKF);
}

std::vector<boost::shared_ptr<Keyframe>> Keyframe::GetBestCovisibilityKeyFrames(
    const int& N) {
  //    unique_lock<mutex> lock (mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    return std::vector<boost::shared_ptr<Keyframe>>(
        mvpOrderedConnectedKeyFrames.begin(),
        mvpOrderedConnectedKeyFrames.begin() + N);
}

void Keyframe::ComputeBoW() {
  if (mBowVec.empty() || mFeatVec.empty()) {
    std::vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from
    // leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  }
}

std::vector<cv::Mat> Keyframe::toDescriptorVector(const cv::Mat& Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

std::vector<boost::shared_ptr<Keyframe>>
Keyframe::GetVectorCovisibleKeyFrames() {
  //    unique_lock<mutex> lock (mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

std::vector<boost::shared_ptr<MapPoint>> Keyframe::GetMapPointMatches() {
  //    unique_lock<mutex> lock (mMutexFeatures);
  return mvpMapPoints;
}

void Keyframe::EraseMapPointMatch(const size_t& idx) {
  //    unique_lock<mutex> lock(mMutexFeatures);
  mvpMapPoints[idx].reset();
}

void Keyframe::EraseMapPointMatch(boost::shared_ptr<MapPoint> pMP) {
  int idx = pMP->GetIndexInKeyFrame(shared_from_this());
  if (idx >= 0) mvpMapPoints[idx].reset();
}

boost::shared_ptr<MapPoint> Keyframe::GetMapPoint(const size_t& idx) {
  //    unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

void Keyframe::ReplaceMapPointMatch(const size_t& idx,
                                    boost::shared_ptr<MapPoint> pMP) {
  mvpMapPoints[idx] = pMP;
}

std::set<boost::shared_ptr<Keyframe>> Keyframe::GetConnectedKeyFrames() {
  //    unique_lock<mutex> lock(mMutexConnections);
  set<boost::shared_ptr<Keyframe>> s;
  for (map<boost::shared_ptr<Keyframe>, int>::iterator mit =
           mConnectedKeyFrameWeights.begin();
       mit != mConnectedKeyFrameWeights.end(); mit++)
    s.insert(mit->first);
  return s;
}

void Keyframe::UndistortKeyPoints() { mvKeysUn = mvKeys; }

void Keyframe::ComputeImageBounds(const cv::Mat& imLeft) {
  mnMinX = 0.0f;
  mnMaxX = imLeft.cols;
  mnMinY = 0.0f;
  mnMaxY = imLeft.rows;
}

Eigen::Vector3d Keyframe::GetCameraCenter() {
  Eigen::Affine3d pose_wc = pose();
  Eigen::Affine3d pose_cw = pose_wc.inverse();
  Eigen::Vector3d Ow = -pose_wc.rotation() * pose_cw.translation();
  return Ow;
}

void Keyframe::SetBadFlag() {
  {
    //    unique_lock<mutex> lock(mMutexConnections);
    if (id() == 0)
      return;
    else if (mbNotErase) {
      mbToBeErased = true;
      return;
    }
  }

  for (map<boost::shared_ptr<Keyframe>, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    mit->first->EraseConnection(shared_from_this());

  for (size_t i = 0; i < mvpMapPoints.size(); i++)
    if (mvpMapPoints[i]) mvpMapPoints[i]->EraseObservation(shared_from_this());
  {
    //    unique_lock<mutex> lock(mMutexConnections);
    //    unique_lock<mutex> lock1(mMutexFeatures);

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();

    // Update Spanning Tree
    set<boost::shared_ptr<Keyframe>> sParentCandidates;
    sParentCandidates.insert(mpParent);

    // Assign at each iteration one children with a parent (the pair with
    // highest covisibility weight)
    // Include that children as new parent candidate for the rest
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int max = -1;
      boost::shared_ptr<Keyframe> pC;
      boost::shared_ptr<Keyframe> pP;

      for (set<boost::shared_ptr<Keyframe>>::iterator
               sit = mspChildrens.begin(),
               send = mspChildrens.end();
           sit != send; sit++) {
        boost::shared_ptr<Keyframe> pKF = *sit;
        if (pKF->isBad()) continue;

        // Check if a parent candidate is connected to the keyframe
        vector<boost::shared_ptr<Keyframe>> vpConnected =
            pKF->GetVectorCovisibleKeyFrames();
        for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
          for (set<boost::shared_ptr<Keyframe>>::iterator
                   spcit = sParentCandidates.begin(),
                   spcend = sParentCandidates.end();
               spcit != spcend; spcit++) {
            if (vpConnected[i]->id() == (*spcit)->id()) {
              int w = pKF->GetWeight(vpConnected[i]);
              if (w > max) {
                pC = pKF;
                pP = vpConnected[i];
                max = w;
                bContinue = true;
              }
            }
          }
        }
      }

      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else
        break;
    }

    // If a children has no covisibility links with any parent candidate, assign
    // to the original parent of this KF
    if (!mspChildrens.empty())
      for (set<boost::shared_ptr<Keyframe>>::iterator sit =
               mspChildrens.begin();
           sit != mspChildrens.end(); sit++) {
        (*sit)->ChangeParent(mpParent);
      }

    mpParent->EraseChild(shared_from_this());
    //    mTcp = Tcw * mpParent->GetPoseInverse();
    mbBad = true;
  }

  mpMap->EraseKeyFrame(shared_from_this());
  mpKeyFrameDB->erase(shared_from_this());
}

void Keyframe::EraseConnection(boost::shared_ptr<Keyframe> pKF) {
  bool bUpdate = false;
  {
    //        unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate) UpdateBestCovisibles();
}

int Keyframe::GetWeight(boost::shared_ptr<Keyframe> pKF) {
  //  unique_lock<mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void Keyframe::ChangeParent(boost::shared_ptr<Keyframe> pKF) {
  //  unique_lock<mutex> lockCon(mMutexConnections);
  mpParent = pKF;
  pKF->AddChild(shared_from_this());
}

void Keyframe::EraseChild(boost::shared_ptr<Keyframe> pKF) {
//  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

} /* namespace dvo_slam */
