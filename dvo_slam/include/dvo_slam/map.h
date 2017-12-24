#ifndef MAP_H
#define MAP_H

#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace dvo_slam {

class MapPoint;
class Keyframe;

class Map {
 public:
  Map();
  virtual ~Map();

  void AddKeyFrame (boost::shared_ptr<Keyframe> pKF);
  void AddMapPoint(boost::shared_ptr<MapPoint> pMP);

 protected:
  std::set<boost::shared_ptr<MapPoint>> mspMapPoints;
  std::set<boost::shared_ptr<Keyframe>> mspKeyFrames;

  std::map<long unsigned int, boost::shared_ptr<MapPoint>> mpMapPoints;
  std::map<long unsigned int, boost::shared_ptr<Keyframe>> mpKeyFrames;

  long unsigned int mnMaxKFid;
};
}

#endif  // MAP_H
