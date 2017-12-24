#include "dvo_slam/map.h"
#include <dvo_slam/keyframe.h>
#include <dvo_slam/mappoint.h>

namespace dvo_slam {

Map* Map::instance = nullptr;

Map::Map() : mnMaxKFid(0) { instance = this; }

Map::~Map() {}

void Map::AddKeyFrame(boost::shared_ptr<Keyframe> pKF) {
  //    unique_lock<mutex> lock (mMutexMap);
  mspKeyFrames.insert(pKF);
  if (pKF->id() > mnMaxKFid) mnMaxKFid = pKF->id();
}

void Map::AddMapPoint(boost::shared_ptr<MapPoint> pMP) {
  //    unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::ReplaceMapPoint(boost::shared_ptr<MapPoint> pMP1,
                          boost::shared_ptr<MapPoint> pMP2) {
  pMP1->Replace(pMP2);
  EraseMapPoint(pMP1);
}

void Map::EraseMapPoint(boost::shared_ptr<MapPoint> pMP) {
//  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}
}
