/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
*of Zaragoza)
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

#include "dvo_slam/KeyFrameDatabase.h"

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "dvo_slam/keyframe.h"

#include <mutex>

using namespace std;

namespace dvo_slam {

KeyframeDatabase::KeyframeDatabase(const boost::shared_ptr<ORBVocabulary> voc)
    : mpVoc(voc) {
  mvInvertedFile.resize(voc->size());
}

void KeyframeDatabase::add(boost::shared_ptr<Keyframe> pKF) {
  unique_lock<mutex> lock(mMutex);

  for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                        vend = pKF->mBowVec.end();
       vit != vend; vit++)
    mvInvertedFile[vit->first].push_back(pKF);
}

void KeyframeDatabase::erase(boost::shared_ptr<Keyframe> pKF) {
  unique_lock<mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                        vend = pKF->mBowVec.end();
       vit != vend; vit++) {
    // List of Keyframes that share the word
    list<boost::shared_ptr<Keyframe>>& lKFs = mvInvertedFile[vit->first];

    for (list<boost::shared_ptr<Keyframe>>::iterator lit = lKFs.begin(),
                                                     lend = lKFs.end();
         lit != lend; lit++) {
      if (pKF == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
}

void KeyframeDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

vector<boost::shared_ptr<Keyframe>> KeyframeDatabase::DetectLoopCandidates(
    boost::shared_ptr<Keyframe> pKF, float minScore) {
  set<boost::shared_ptr<Keyframe>> spConnectedKeyframes =
      pKF->GetConnectedKeyFrames();
  list<boost::shared_ptr<Keyframe>> lKFsSharingWords;

  // Search all Keyframes that share a word with current Keyframes
  // Discard Keyframes connected to the query Keyframe
  {
    unique_lock<mutex> lock(mMutex);

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                          vend = pKF->mBowVec.end();
         vit != vend; vit++) {
      list<boost::shared_ptr<Keyframe>>& lKFs = mvInvertedFile[vit->first];

      for (list<boost::shared_ptr<Keyframe>>::iterator lit = lKFs.begin(),
                                                       lend = lKFs.end();
           lit != lend; lit++) {
        boost::shared_ptr<Keyframe> pKFi = *lit;
        if (pKFi->mnLoopQuery != pKF->id()) {
          pKFi->mnLoopWords = 0;
          if (!spConnectedKeyframes.count(pKFi)) {
            pKFi->mnLoopQuery = pKF->id();
            lKFsSharingWords.push_back(pKFi);
          }
        }
        pKFi->mnLoopWords++;
      }
    }
  }

  if (lKFsSharingWords.empty()) return vector<boost::shared_ptr<Keyframe>>();

  list<pair<float, boost::shared_ptr<Keyframe>>> lScoreAndMatch;

  // Only compare against those Keyframes that share enough words
  int maxCommonWords = 0;
  for (list<boost::shared_ptr<Keyframe>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    if ((*lit)->mnLoopWords > maxCommonWords)
      maxCommonWords = (*lit)->mnLoopWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  int nscores = 0;

  // Compute similarity score. Retain the matches whose score is higher than
  // minScore
  for (list<boost::shared_ptr<Keyframe>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    boost::shared_ptr<Keyframe> pKFi = *lit;

    if (pKFi->mnLoopWords > minCommonWords) {
      nscores++;

      float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

      pKFi->mLoopScore = si;
      if (si >= minScore) lScoreAndMatch.push_back(make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return vector<boost::shared_ptr<Keyframe>>();

  list<pair<float, boost::shared_ptr<Keyframe>>> lAccScoreAndMatch;
  float bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  for (list<pair<float, boost::shared_ptr<Keyframe>>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    boost::shared_ptr<Keyframe> pKFi = it->second;
    vector<boost::shared_ptr<Keyframe>> vpNeighs =
        pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = it->first;
    boost::shared_ptr<Keyframe> pBestKF = pKFi;
    for (vector<boost::shared_ptr<Keyframe>>::iterator vit = vpNeighs.begin(),
                                                       vend = vpNeighs.end();
         vit != vend; vit++) {
      boost::shared_ptr<Keyframe> pKF2 = *vit;
      if (pKF2->mnLoopQuery == pKF->id() &&
          pKF2->mnLoopWords > minCommonWords) {
        accScore += pKF2->mLoopScore;
        if (pKF2->mLoopScore > bestScore) {
          pBestKF = pKF2;
          bestScore = pKF2->mLoopScore;
        }
      }
    }

    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those Keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;

  set<boost::shared_ptr<Keyframe>> spAlreadyAddedKF;
  vector<boost::shared_ptr<Keyframe>> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  for (list<pair<float, boost::shared_ptr<Keyframe>>>::iterator
           it = lAccScoreAndMatch.begin(),
           itend = lAccScoreAndMatch.end();
       it != itend; it++) {
    if (it->first > minScoreToRetain) {
      boost::shared_ptr<Keyframe> pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpLoopCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpLoopCandidates;
}

// vector<boost::shared_ptr<Keyframe>>
// KeyframeDatabase::DetectRelocalizationCandidates (boost::shared_ptr<Frame> F)
//{
//    list<boost::shared_ptr<Keyframe>> lKFsSharingWords;

//    // Search all Keyframes that share a word with current frame
//    {
//        unique_lock<mutex> lock (mMutex);

//        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin (),
//                                              vend = F->mBowVec.end ();
//             vit != vend;
//             vit++)
//        {
//            list<boost::shared_ptr<Keyframe>>& lKFs =
//            mvInvertedFile[vit->first];

//            for (list<boost::shared_ptr<Keyframe>>::iterator lit = lKFs.begin
//            (), lend = lKFs.end ();
//                 lit != lend;
//                 lit++)
//            {
//                boost::shared_ptr<Keyframe> pKFi = *lit;

//                if (pKFi->mnRelocQuery != F->mnId)
//                {
//                    pKFi->mnRelocWords = 0;
//                    pKFi->mnRelocQuery = F->mnId;
//                    lKFsSharingWords.push_back (pKFi);
//                }
//                pKFi->mnRelocWords++;
//            }
//        }
//    }

//    if (lKFsSharingWords.empty ()) return
//    vector<boost::shared_ptr<Keyframe>>();

//    // Only compare against those Keyframes that share enough words
//    int maxCommonWords = 0;
//    for (list<boost::shared_ptr<Keyframe>>::iterator lit =
//    lKFsSharingWords.begin (),
//                                   lend = lKFsSharingWords.end ();
//         lit != lend;
//         lit++)
//    {
//        if ((*lit)->mnRelocWords > maxCommonWords)
//            maxCommonWords = (*lit)->mnRelocWords;
//    }

//    int minCommonWords = maxCommonWords * 0.8f;

//    list<pair<float, boost::shared_ptr<Keyframe>>> lScoreAndMatch;

//    int nscores = 0;

//    // Compute similarity score.
//    for (list<boost::shared_ptr<Keyframe>>::iterator lit =
//    lKFsSharingWords.begin (),
//                                   lend = lKFsSharingWords.end ();
//         lit != lend;
//         lit++)
//    {
//        boost::shared_ptr<Keyframe> pKFi = *lit;

//        if (pKFi->mnRelocWords > minCommonWords)
//        {
//            nscores++;
//            float si = mpVoc->score (F->mBowVec, pKFi->mBowVec);
//            pKFi->mRelocScore = si;
//            lScoreAndMatch.push_back (make_pair (si, pKFi));
//        }
//    }

//    if (lScoreAndMatch.empty ()) return vector<boost::shared_ptr<Keyframe>>();

//    list<pair<float, boost::shared_ptr<Keyframe>>> lAccScoreAndMatch;
//    float bestAccScore = 0;

//    // Lets now accumulate score by covisibility
//    for (list<pair<float, boost::shared_ptr<Keyframe>>>::iterator it =
//    lScoreAndMatch.begin (),
//                                                itend = lScoreAndMatch.end ();
//         it != itend;
//         it++)
//    {
//        boost::shared_ptr<Keyframe> pKFi = it->second;
//        vector<boost::shared_ptr<Keyframe>> vpNeighs =
//        pKFi->GetBestCovisibilityKeyframes (10);

//        float bestScore = it->first;
//        float accScore = bestScore;
//        boost::shared_ptr<Keyframe> pBestKF = pKFi;
//        for (vector<boost::shared_ptr<Keyframe>>::iterator vit =
//        vpNeighs.begin (), vend = vpNeighs.end ();
//             vit != vend;
//             vit++)
//        {
//            boost::shared_ptr<Keyframe> pKF2 = *vit;
//            if (pKF2->mnRelocQuery != F->mnId) continue;

//            accScore += pKF2->mRelocScore;
//            if (pKF2->mRelocScore > bestScore)
//            {
//                pBestKF = pKF2;
//                bestScore = pKF2->mRelocScore;
//            }
//        }
//        lAccScoreAndMatch.push_back (make_pair (accScore, pBestKF));
//        if (accScore > bestAccScore) bestAccScore = accScore;
//    }

//    // Return all those Keyframes with a score higher than 0.75*bestScore
//    float minScoreToRetain = 0.75f * bestAccScore;
//    set<boost::shared_ptr<Keyframe>> spAlreadyAddedKF;
//    vector<boost::shared_ptr<Keyframe>> vpRelocCandidates;
//    vpRelocCandidates.reserve (lAccScoreAndMatch.size ());
//    for (list<pair<float, boost::shared_ptr<Keyframe>>>::iterator it =
//    lAccScoreAndMatch.begin (),
//                                                itend = lAccScoreAndMatch.end
//                                                ();
//         it != itend;
//         it++)
//    {
//        const float& si = it->first;
//        if (si > minScoreToRetain)
//        {
//            boost::shared_ptr<Keyframe> pKFi = it->second;
//            if (!spAlreadyAddedKF.count (pKFi))
//            {
//                vpRelocCandidates.push_back (pKFi);
//                spAlreadyAddedKF.insert (pKFi);
//            }
//        }
//    }

//    return vpRelocCandidates;
//}

// vector<boost::shared_ptr<Keyframe>>
// KeyframeDatabase::DetectMyRelocalizationCandidates (boost::shared_ptr<Frame>
// F)
//{
//    list<boost::shared_ptr<Keyframe>> lKFsSharingWords;

//    // Search all Keyframes that share a word with current frame
//    {
//        unique_lock<mutex> lock (mMutex);

//        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin (),
//                                              vend = F->mBowVec.end ();
//             vit != vend;
//             vit++)
//        {
//            list<boost::shared_ptr<Keyframe>>& lKFs =
//            mvInvertedFile[vit->first];

//            for (list<boost::shared_ptr<Keyframe>>::iterator lit = lKFs.begin
//            (), lend = lKFs.end ();
//                 lit != lend;
//                 lit++)
//            {
//                boost::shared_ptr<Keyframe> pKFi = *lit;

//                if (pKFi->mnRelocQuery != F->mnId)
//                {
//                    pKFi->mnRelocWords = 0;
//                    pKFi->mnRelocQuery = F->mnId;
//                    lKFsSharingWords.push_back (pKFi);
//                }
//                pKFi->mnRelocWords++;
//            }
//        }
//    }

//    if (lKFsSharingWords.empty ()) return
//    vector<boost::shared_ptr<Keyframe>>();

//    // Only compare against those Keyframes that share enough words
//    int maxCommonWords = 0;
//    for (list<boost::shared_ptr<Keyframe>>::iterator lit =
//    lKFsSharingWords.begin (),
//                                   lend = lKFsSharingWords.end ();
//         lit != lend;
//         lit++)
//    {
//        if ((*lit)->mnRelocWords > maxCommonWords)
//            maxCommonWords = (*lit)->mnRelocWords;
//    }

//    int minCommonWords = maxCommonWords * 0.8f;

//    vector<boost::shared_ptr<Keyframe>> vpRelocCandidates;

//    for (list<boost::shared_ptr<Keyframe>>::iterator lit =
//    lKFsSharingWords.begin (),
//                                   lend = lKFsSharingWords.end ();
//         lit != lend;
//         lit++)
//    {
//        boost::shared_ptr<Keyframe> pKFi = *lit;
//        if (pKFi->mnRelocWords > minCommonWords)
//            vpRelocCandidates.push_back (pKFi);
//    }

//    return vpRelocCandidates;

//    //    list<pair<float,Keyframe*> > lScoreAndMatch;

//    //    int nscores=0;

//    //    // Compute similarity score.
//    //    for(list<Keyframe*>::iterator lit=lKFsSharingWords.begin(), lend=
//    //    lKFsSharingWords.end(); lit!=lend; lit++)
//    //    {
//    //        Keyframe* pKFi = *lit;

//    //        if(pKFi->mnRelocWords>minCommonWords)
//    //        {
//    //            nscores++;
//    //            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
//    //            pKFi->mRelocScore=si;
//    //            lScoreAndMatch.push_back(make_pair(si,pKFi));
//    //        }
//    //    }

//    //    if(lScoreAndMatch.empty())
//    //        return vector<Keyframe*>();

//    //    list<pair<float,Keyframe*> > lAccScoreAndMatch;
//    //    float bestAccScore = 0;

//    // Lets now accumulate score by covisibility
//    //    for(list<pair<float,Keyframe*> >::iterator
//    it=lScoreAndMatch.begin(),
//    //    itend=lScoreAndMatch.end(); it!=itend; it++)
//    //    {
//    //        Keyframe* pKFi = it->second;
//    //        vector<Keyframe*> vpNeighs =
//    //        pKFi->GetBestCovisibilityKeyframes(10);

//    //        float bestScore = it->first;
//    //        float accScore = bestScore;
//    //        Keyframe* pBestKF = pKFi;
//    //        for(vector<Keyframe*>::iterator vit=vpNeighs.begin(),
//    //        vend=vpNeighs.end(); vit!=vend; vit++)
//    //        {
//    //            Keyframe* pKF2 = *vit;
//    //            if(pKF2->mnRelocQuery!=F->mnId)
//    //                continue;

//    //            accScore+=pKF2->mRelocScore;
//    //            if(pKF2->mRelocScore>bestScore)
//    //            {
//    //                pBestKF=pKF2;
//    //                bestScore = pKF2->mRelocScore;
//    //            }

//    //        }
//    //        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
//    //        if(accScore>bestAccScore)
//    //            bestAccScore=accScore;
//    //    }

//    // Return all those Keyframes with a score higher than 0.75*bestScore
//    //    float minScoreToRetain = 0.75f*bestAccScore;
//    //    set<Keyframe*> spAlreadyAddedKF;
//    //    vector<Keyframe*> vpRelocCandidates;
//    //    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
//    //    for(list<pair<float,Keyframe*> >::iterator
//    //    it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end();
//    //    it!=itend; it++)
//    //    {
//    //        const float &si = it->first;
//    //        if(si>minScoreToRetain)
//    //        {
//    //            Keyframe* pKFi = it->second;
//    //            if(!spAlreadyAddedKF.count(pKFi))
//    //            {
//    //                vpRelocCandidates.push_back(pKFi);
//    //                spAlreadyAddedKF.insert(pKFi);
//    //            }
//    //        }
//    //    }

//    //    return vpRelocCandidates;
//}

vector<boost::shared_ptr<Keyframe>> KeyframeDatabase::DetectMatchedKeyFrames(
    boost::shared_ptr<Keyframe> pKF) {
  list<boost::shared_ptr<Keyframe>> lKFsSharingWords;

  // Search all Keyframes that share a word with current frame
  {
    unique_lock<mutex> lock(mMutex);

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                          vend = pKF->mBowVec.end();
         vit != vend; vit++) {
      list<boost::shared_ptr<Keyframe>>& lKFs = mvInvertedFile[vit->first];

      for (list<boost::shared_ptr<Keyframe>>::iterator lit = lKFs.begin(),
                                                       lend = lKFs.end();
           lit != lend; lit++) {
        boost::shared_ptr<Keyframe> pKFi = *lit;

        if (pKFi->mnRelocQuery != pKF->id()) {
          pKFi->mnRelocWords = 0;
          pKFi->mnRelocQuery = pKF->id();
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnRelocWords++;
      }
    }
  }

  if (lKFsSharingWords.empty()) return vector<boost::shared_ptr<Keyframe>>();

  // Only compare against those Keyframes that share enough words
  int maxCommonWords = 0;
  for (list<boost::shared_ptr<Keyframe>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    if ((*lit)->mnRelocWords > maxCommonWords)
      maxCommonWords = (*lit)->mnRelocWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  vector<boost::shared_ptr<Keyframe>> vpRelocCandidates;

  for (list<boost::shared_ptr<Keyframe>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    boost::shared_ptr<Keyframe> pKFi = *lit;
    if (pKFi->mnRelocWords > minCommonWords) vpRelocCandidates.push_back(pKFi);
  }

  return vpRelocCandidates;
}

void KeyframeDatabase::SetInvertedFile(
    std::vector<std::list<boost::shared_ptr<Keyframe>>> vInvertedFile) {
  unique_lock<mutex> lock(mMutex);
  mvInvertedFile = vInvertedFile;
}

std::vector<std::list<boost::shared_ptr<Keyframe>>>&
KeyframeDatabase::GetInvertedFile() {
  unique_lock<mutex> lock(mMutex);
  return mvInvertedFile;
}

void KeyframeDatabase::SetInvertedFileVec(
    std::vector<std::list<unsigned long>> vInvertedFileVec) {
  unique_lock<mutex> lock(mMutex);
  mvInvertedFileVec = vInvertedFileVec;
}

std::vector<std::list<long unsigned int>>
KeyframeDatabase::GetInvertedFileVec() {
  unique_lock<mutex> lock(mMutex);

  std::vector<std::list<long unsigned int>> vInvertedFileVec;
  for (auto v : mvInvertedFile) {
    std::list<long unsigned int> l;
    for (auto le : v) {
      l.push_back(le->id());
    }

    vInvertedFileVec.push_back(l);
  }

  return vInvertedFileVec;
}

}  // namespace ORB_SLAM
