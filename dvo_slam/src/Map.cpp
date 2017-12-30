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

#include "dvo_slam/Map.h"
#include "dvo_slam/keyframe.h"
#include "dvo_slam/mappoint.h"

#include <mutex>

namespace dvo_slam {

Map* Map::instance = nullptr;

Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) { instance = this; }

void Map::AddKeyFrame(boost::shared_ptr<Keyframe> pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.insert(pKF);
  if (pKF->id() > mnMaxKFid) mnMaxKFid = pKF->id();
}

void Map::AddMapPoint(boost::shared_ptr<MapPoint> pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(boost::shared_ptr<MapPoint> pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(boost::shared_ptr<Keyframe> pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.erase(pKF);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(
    const std::vector<boost::shared_ptr<MapPoint>>& vpMPs) {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

vector<boost::shared_ptr<Keyframe>> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<boost::shared_ptr<Keyframe>>(mspKeyFrames.begin(),
                                             mspKeyFrames.end());
}

std::vector<boost::shared_ptr<MapPoint>> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<boost::shared_ptr<MapPoint>>(mspMapPoints.begin(),
                                             mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

std::vector<boost::shared_ptr<MapPoint>> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

void Map::clear() {
  for (auto entry : mspMapPoints) entry.reset();

  for (auto entry : mspKeyFrames) entry.reset();

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

void Map::ReplaceMapPoint(boost::shared_ptr<MapPoint> pMP1,
                          boost::shared_ptr<MapPoint> pMP2) {
  pMP1->Replace(pMP2);
  EraseMapPoint(pMP1);
}

}  // namespace ORB_SLAM
