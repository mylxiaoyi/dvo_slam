/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/RSLAM>
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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <list>
#include <set>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <dvo_slam/orbvocabulary.h>

#include <memory>
#include <mutex>

namespace dvo_slam
{

class Keyframe;

class KeyframeDatabase
{
public:
    KeyframeDatabase (const boost::shared_ptr<ORBVocabulary> voc);

    void add (boost::shared_ptr<Keyframe> pKF);

    void erase (boost::shared_ptr<Keyframe> pKF);

    void clear ();

    // Loop Detection
    vector<boost::shared_ptr<Keyframe>>
    DetectLoopCandidates (boost::shared_ptr<Keyframe> pKF, float minScore);

    // Relocalization
//    std::vector<boost::shared_ptr<KeyFrame>>
//    DetectRelocalizationCandidates (boost::shared_ptr<Frame> F);

//    vector<boost::shared_ptr<KeyFrame>>
//    DetectMyRelocalizationCandidates (boost::shared_ptr<Frame> F);

    vector<boost::shared_ptr<Keyframe>>
    DetectMatchedKeyFrames (boost::shared_ptr<Keyframe> pKF);

    std::vector<std::list<long unsigned int>> GetInvertedFileVec ();
    void SetInvertedFileVec (std::vector<std::list<long unsigned int>> vInvertedFileVec);
    void SetInvertedFile (std::vector<std::list<boost::shared_ptr<Keyframe>>> vInvertedFile);
    std::vector<std::list<boost::shared_ptr<Keyframe>>>& GetInvertedFile ();

protected:
    // Associated vocabulary
    const boost::shared_ptr<ORBVocabulary> mpVoc;

    // Inverted file
    std::vector<list<boost::shared_ptr<Keyframe>>> mvInvertedFile;
    std::vector<list<long unsigned int>> mvInvertedFileVec;

    // Mutex
    std::mutex mMutex;
};

} // namespace ORB_SLAM

#endif
