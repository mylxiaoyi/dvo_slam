#include "dvo_slam/map.h"
#include <dvo_slam/mappoint.h>
#include <dvo_slam/keyframe.h>

namespace dvo_slam {

Map::Map() : mnMaxKFid(0) {}

Map::~Map() {}

void Map::AddKeyFrame (boost::shared_ptr<Keyframe> pKF)
{
//    unique_lock<mutex> lock (mMutexMap);
    mspKeyFrames.insert (pKF);
    if (pKF->id() > mnMaxKFid) mnMaxKFid = pKF->id();
}

void Map::AddMapPoint(boost::shared_ptr<MapPoint> pMP)
{
//    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}
}
