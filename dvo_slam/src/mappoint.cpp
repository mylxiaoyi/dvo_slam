#include "dvo_slam/mappoint.h"
#include "dvo_slam/ORBmatcher.h"

namespace dvo_slam {

long unsigned int MapPoint::mnNextId = 0;

MapPoint::MapPoint() : mbBad(false) {}

MapPoint::MapPoint(Eigen::Vector3d pos, KeyframePtr ref)
    : pos_(pos), ref_(ref), mbBad(false), nObs(0) {
  mnId = mnNextId++;
}

MapPoint::~MapPoint() {}

cv::Mat MapPoint::GetDescriptor() { return mDescriptor.clone(); }

void MapPoint::AddObservation(KeyframePtr keyframe, size_t idx) {
  if (mObservations.count(keyframe)) return;
  mObservations[keyframe] = idx;
  nObs += 2;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  std::vector<cv::Mat> vDescriptors;

  std::map<boost::shared_ptr<Keyframe>, size_t> observations;

  {
    //    unique_lock<mutex> lock1(mMutexFeatures);
    if (mbBad) return;
    observations = mObservations;
  }

  if (observations.empty()) return;

  vDescriptors.reserve(observations.size());

  for (std::map<boost::shared_ptr<Keyframe>, size_t>::iterator
           mit = observations.begin(),
           mend = observations.end();
       mit != mend; mit++) {
    boost::shared_ptr<Keyframe> pKF = mit->first;

    if (!pKF->isBad())
      vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
  }

  if (vDescriptors.empty()) return;

  // Compute distances between them
  const size_t N = vDescriptors.size();

  float Distances[N][N];
  for (size_t i = 0; i < N; i++) {
    Distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++) {
      int distij =
          ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
      Distances[i][j] = distij;
      Distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for (size_t i = 0; i < N; i++) {
    std::vector<int> vDists(Distances[i], Distances[i] + N);
    std::sort(vDists.begin(), vDists.end());
    int median = vDists[0.5 * (N - 1)];

    if (median < BestMedian) {
      BestMedian = median;
      BestIdx = i;
    }
  }

  {
    //    unique_lock<mutex> lock(mMutexFeatures);
    mDescriptor = vDescriptors[BestIdx].clone();
  }
}

// void MapPoint::UpdateNormalAndDepth() {
//  std::map<boost::shared_ptr<Keyframe>, size_t> observations;
//  boost::shared_ptr<Keyframe> pRefKF;
////  cv::Mat Pos;
//  Eigen::Vector3d Pos;
//  {
////    unique_lock<mutex> lock1(mMutexFeatures);
////    unique_lock<mutex> lock2(mMutexPos);
//    if (mbBad) return;
//    observations = mObservations;
////    pRefKF = mpRefKF;
//    pRefKF = ref_;
////    Pos = mWorldPos.clone();
//    Pos = pos_;
//  }

//  if (observations.empty()) return;

////  cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
//  Eigen::Vector3d normal;
//  int n = 0;
//  for (std::map<boost::shared_ptr<KeyFrame>, size_t>::iterator
//           mit = observations.begin(),
//           mend = observations.end();
//       mit != mend; mit++) {
//    boost::shared_ptr<Keyframe> pKF = mit->first;
//    cv::Mat Owi = pKF->GetCameraCenter();
//    cv::Mat normali = mWorldPos - Owi;
//    normal = normal + normali / cv::norm(normali);
//    n++;
//  }

//  cv::Mat PC = Pos - pRefKF->GetCameraCenter();
//  const float dist = cv::norm(PC);
//  const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
//  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
//  const int nLevels = pRefKF->mnScaleLevels;

//  {
//    unique_lock<mutex> lock3(mMutexPos);
//    mfMaxDistance = dist * levelScaleFactor;
//    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
//    mNormalVector = normal / n;
//  }
//}

int MapPoint::GetIndexInKeyFrame(boost::shared_ptr<Keyframe> pKF) {
  //    unique_lock<mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF))
    return mObservations[pKF];
  else
    return -1;
}

void MapPoint::EraseObservation(boost::shared_ptr<Keyframe> pKF) {
  bool bBad = false;
  {
//    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF)) {
      int idx = mObservations[pKF];
      nObs -= 2;

      mObservations.erase(pKF);

      if (ref_ == pKF) ref_ = mObservations.begin()->first;

      // If only 2 observations or less, discard point
      if (nObs <= 2) bBad = true;
    }
  }

//  if (bBad) SetBadFlag();
  mbBad = bBad;
}
}
