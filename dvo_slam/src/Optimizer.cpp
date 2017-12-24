﻿/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
*of Zaragoza)
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

#include "dvo_slam/Optimizer.h"
#include "dvo_slam/keyframe.h"
#include "dvo_slam/map.h"
#include "dvo_slam/mappoint.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

//#include <Eigen/StdVector>

//#include "Converter.h"

//#include <mutex>

//#include "utilite/UVar.h"

namespace dvo_slam {

g2o::SE3Quat toSE3Quat(Eigen::Affine3d pose) {
  Eigen::Matrix<double, 3, 3> R = pose.rotation();
  Eigen::Matrix<double, 3, 1> t = pose.translation();
  return g2o::SE3Quat(R, t);
}

Eigen::Affine3d toAffine(g2o::SE3Quat pose) {
  Eigen::Affine3d p;
  p.matrix() = pose.to_homogeneous_matrix();
  return p;
}
// void Optimizer::GlobalBundleAdjustemnt (std::shared_ptr<Map> pMap,
//                                        int nIterations,
//                                        bool* pbStopFlag,
//                                        const unsigned long nLoopKF,
//                                        const bool bRobust)
//{
//    vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames ();
//    vector<std::shared_ptr<MapPoint>> vpMP = pMap->GetAllMapPoints ();
//    BundleAdjustment (vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
//}

// void Optimizer::BundleAdjustment (const vector<std::shared_ptr<KeyFrame>>&
// vpKFs,
//                                  const vector<std::shared_ptr<MapPoint>>&
//                                  vpMP,
//                                  int nIterations,
//                                  bool* pbStopFlag,
//                                  const unsigned long nLoopKF,
//                                  const bool bRobust)
//{
//    vector<bool> vbNotIncludedMP;
//    vbNotIncludedMP.resize (vpMP.size ());

//    g2o::SparseOptimizer optimizer;
//    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

//    linearSolver = new
//    g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> ();

//    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3
//    (linearSolver);

//    g2o::OptimizationAlgorithmLevenberg* solver =
//    new g2o::OptimizationAlgorithmLevenberg (solver_ptr);
//    optimizer.setAlgorithm (solver);

//    if (pbStopFlag) optimizer.setForceStopFlag (pbStopFlag);

//    long unsigned int maxKFid = 0;

//    // Set KeyFrame vertices
//    for (size_t i = 0; i < vpKFs.size (); i++)
//    {
//        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
//        if (pKF->isBad ()) continue;
//        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap ();
//        vSE3->setEstimate (Converter::toSE3Quat (pKF->GetPose ()));
//        vSE3->setId (pKF->mnId);
//        vSE3->setFixed (pKF->mnId == 0);
//        optimizer.addVertex (vSE3);
//        if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
//    }

//    const float thHuber2D = sqrt (5.99);
//    const float thHuber3D = sqrt (7.815);

//    // Set MapPoint vertices
//    for (size_t i = 0; i < vpMP.size (); i++)
//    {
//        std::shared_ptr<MapPoint> pMP = vpMP[i];
//        if (pMP->isBad ()) continue;
//        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ ();
//        vPoint->setEstimate (Converter::toVector3d (pMP->GetWorldPos ()));
//        const int id = pMP->mnId + maxKFid + 1;
//        vPoint->setId (id);
//        vPoint->setMarginalized (true);
//        optimizer.addVertex (vPoint);

//        const map<std::shared_ptr<KeyFrame>, size_t> observations =
//        pMP->GetObservations ();

//        int nEdges = 0;
//        // SET EDGES
//        for (map<std::shared_ptr<KeyFrame>, size_t>::const_iterator mit =
//             observations.begin ();
//             mit != observations.end (); mit++)
//        {

//            std::shared_ptr<KeyFrame> pKF = mit->first;
//            if (pKF->isBad () || pKF->mnId > maxKFid) continue;

//            nEdges++;

//            const cv::KeyPoint& kpUn = pKF->mvKeysUn[mit->second];

//            if (pKF->mvuRight[mit->second] < 0)
//            {
//                Eigen::Matrix<double, 2, 1> obs;
//                obs << kpUn.pt.x, kpUn.pt.y;

//                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ ();

//                e->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//                (
//                                 optimizer.vertex (id)));
//                e->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//                (
//                                 optimizer.vertex (pKF->mnId)));
//                e->setMeasurement (obs);
//                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
//                e->setInformation (Eigen::Matrix2d::Identity () * invSigma2);

//                if (bRobust)
//                {
//                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                    e->setRobustKernel (rk);
//                    rk->setDelta (thHuber2D);
//                }

//                e->fx = pKF->fx;
//                e->fy = pKF->fy;
//                e->cx = pKF->cx;
//                e->cy = pKF->cy;

//                optimizer.addEdge (e);
//            }
//            else
//            {
//                Eigen::Matrix<double, 3, 1> obs;
//                const float kp_ur = pKF->mvuRight[mit->second];
//                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

//                g2o::EdgeStereoSE3ProjectXYZ* e = new
//                g2o::EdgeStereoSE3ProjectXYZ ();

//                e->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//                (
//                                 optimizer.vertex (id)));
//                e->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//                (
//                                 optimizer.vertex (pKF->mnId)));
//                e->setMeasurement (obs);
//                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
//                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity () *
//                invSigma2;
//                e->setInformation (Info);

//                if (bRobust)
//                {
//                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                    e->setRobustKernel (rk);
//                    rk->setDelta (thHuber3D);
//                }

//                e->fx = pKF->fx;
//                e->fy = pKF->fy;
//                e->cx = pKF->cx;
//                e->cy = pKF->cy;
//                e->bf = pKF->mbf;

//                optimizer.addEdge (e);
//            }
//        }

//        if (nEdges == 0)
//        {
//            optimizer.removeVertex (vPoint);
//            vbNotIncludedMP[i] = true;
//        }
//        else
//        {
//            vbNotIncludedMP[i] = false;
//        }
//    }

//    // Optimize!
//    optimizer.initializeOptimization ();
//    optimizer.optimize (nIterations);

//    // Recover optimized data

//    // Keyframes
//    for (size_t i = 0; i < vpKFs.size (); i++)
//    {
//        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
//        if (pKF->isBad ()) continue;
//        g2o::VertexSE3Expmap* vSE3 =
//        static_cast<g2o::VertexSE3Expmap*> (optimizer.vertex (pKF->mnId));
//        g2o::SE3Quat SE3quat = vSE3->estimate ();
//        if (nLoopKF == 0)
//        {
//            pKF->SetPose (Converter::toCvMat (SE3quat));
//        }
//        else
//        {
//            pKF->mTcwGBA.create (4, 4, CV_32F);
//            Converter::toCvMat (SE3quat).copyTo (pKF->mTcwGBA);
//            pKF->mnBAGlobalForKF = nLoopKF;
//        }
//    }

//    // Points
//    for (size_t i = 0; i < vpMP.size (); i++)
//    {
//        if (vbNotIncludedMP[i]) continue;

//        std::shared_ptr<MapPoint> pMP = vpMP[i];

//        if (pMP->isBad ()) continue;
//        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>
//        (
//        optimizer.vertex (pMP->mnId + maxKFid + 1));

//        if (nLoopKF == 0)
//        {
//            pMP->SetWorldPos (Converter::toCvMat (vPoint->estimate ()));
//            pMP->UpdateNormalAndDepth ();
//        }
//        else
//        {
//            pMP->mPosGBA.create (3, 1, CV_32F);
//            Converter::toCvMat (vPoint->estimate ()).copyTo (pMP->mPosGBA);
//            pMP->mnBAGlobalForKF = nLoopKF;
//        }
//    }
//}

int Optimizer::PoseOptimization(boost::shared_ptr<Keyframe> pFrame) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  //  vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
  vSE3->setEstimate(toSE3Quat(pFrame->pose().inverse()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;

  std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  std::vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  //  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815);

  {
    //    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      boost::shared_ptr<MapPoint> pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Stereo observation
        {
          nInitialCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          // SET EDGE
          Eigen::Matrix<double, 3, 1> obs;
          //          const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
          const cv::KeyPoint& kpUn = pFrame->mvKeys[i];
          const float& kp_ur = pFrame->mvuRight[i];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
              new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(0)));
          e->setMeasurement(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaStereo);

          e->fx = pFrame->fx;
          e->fy = pFrame->fy;
          e->cx = pFrame->cx;
          e->cy = pFrame->cy;
          e->bf = pFrame->mbf;
          Eigen::Vector3d Xw = pMP->GetPos();
          e->Xw[0] = Xw[0];
          e->Xw[1] = Xw[1];
          e->Xw[2] = Xw[2];

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }
      }
    }
  }

  if (nInitialCorrespondences < 3) return 0;

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end
  // they can be classified as inliers again.
  //  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  for (size_t it = 0; it < 4; it++) {
    vSE3->setEstimate(toSE3Quat(pFrame->pose()));
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        pFrame->mvbOutlier[idx] = false;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    if (optimizer.edges().size() < 10) break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Eigen::Affine3d pose = toAffine(SE3quat_recov);
  pFrame->pose(pose.inverse());

  return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(boost::shared_ptr<Keyframe> pKF,
                                      bool* pbStopFlag,
                                      boost::shared_ptr<Map> pMap) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  std::list<boost::shared_ptr<Keyframe>> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->id();

  const std::vector<boost::shared_ptr<Keyframe>> vNeighKFs =
      pKF->GetVectorCovisibleKeyFrames();
  for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
    boost::shared_ptr<Keyframe> pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->id();
    if (!pKFi->isBad()) lLocalKeyFrames.push_back(pKFi);
  }

  // Local MapPoints seen in Local KeyFrames
  std::list<boost::shared_ptr<MapPoint>> lLocalMapPoints;
  for (std::list<boost::shared_ptr<Keyframe>>::iterator
           lit = lLocalKeyFrames.begin(),
           lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    std::vector<boost::shared_ptr<MapPoint>> vpMPs =
        (*lit)->GetMapPointMatches();
    for (vector<boost::shared_ptr<MapPoint>>::iterator vit = vpMPs.begin(),
                                                     vend = vpMPs.end();
         vit != vend; vit++) {
      boost::shared_ptr<MapPoint> pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->id()) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->id();
          }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
  // Local Keyframes
  std::list<boost::shared_ptr<Keyframe>> lFixedCameras;
  for (std::list<boost::shared_ptr<MapPoint>>::iterator
           lit = lLocalMapPoints.begin(),
           lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    std::map<boost::shared_ptr<Keyframe>, size_t> observations =
        (*lit)->GetObservations();
    for (std::map<boost::shared_ptr<Keyframe>, size_t>::iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      boost::shared_ptr<Keyframe> pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->id() &&
          pKFi->mnBAFixedForKF != pKF->id()) {
        pKFi->mnBAFixedForKF = pKF->id();
        if (!pKFi->isBad()) lFixedCameras.push_back(pKFi);
      }
    }
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // Set Local KeyFrame vertices
  for (std::list<boost::shared_ptr<Keyframe>>::iterator
           lit = lLocalKeyFrames.begin(),
           lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    boost::shared_ptr<Keyframe> pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat(pKFi->pose().inverse()));
    vSE3->setId(pKFi->id());
    vSE3->setFixed(pKFi->id() == 1);
    optimizer.addVertex(vSE3);
    if (pKFi->id() > maxKFid) maxKFid = pKFi->id();
  }

  // Set Fixed KeyFrame vertices
  for (std::list<boost::shared_ptr<Keyframe>>::iterator
           lit = lFixedCameras.begin(),
           lend = lFixedCameras.end();
       lit != lend; lit++) {
    boost::shared_ptr<Keyframe> pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat(pKFi->pose().inverse()));
    vSE3->setId(pKFi->id());
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->id() > maxKFid) maxKFid = pKFi->id();
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<boost::shared_ptr<Keyframe>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<boost::shared_ptr<MapPoint>> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  //  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  for (std::list<boost::shared_ptr<MapPoint>>::iterator
           lit = lLocalMapPoints.begin(),
           lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    boost::shared_ptr<MapPoint> pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetPos());
    int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<boost::shared_ptr<Keyframe>, size_t> observations =
        pMP->GetObservations();

    // Set edges
    for (std::map<boost::shared_ptr<Keyframe>, size_t>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      boost::shared_ptr<Keyframe> pKFi = mit->first;

      if (!pKFi->isBad()) {
        const cv::KeyPoint& kpUn = pKFi->mvKeys[mit->second];

        // Stereo observation
        {
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[mit->second];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->id())));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          e->fx = pKFi->fx;
          e->fy = pKFi->fy;
          e->cx = pKFi->cx;
          e->cy = pKFi->cy;
          e->bf = pKFi->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag) bDoMore = false;

  if (bDoMore) {
    // Check inlier observations

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      boost::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
      }

      e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  std::vector<
      std::pair<boost::shared_ptr<Keyframe>, boost::shared_ptr<MapPoint>>>
      vToErase;
  vToErase.reserve(vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    boost::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      boost::shared_ptr<Keyframe> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::pair<boost::shared_ptr<Keyframe>, boost::shared_ptr<MapPoint>>(pKFi, pMP));
    }
  }

  // Get Map Mutex
  //  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      boost::shared_ptr<Keyframe> pKFi = vToErase[i].first;
      boost::shared_ptr<MapPoint> pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data

  // Keyframes
  for (list<boost::shared_ptr<Keyframe>>::iterator lit = lLocalKeyFrames.begin(),
                                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    boost::shared_ptr<Keyframe> pKF = *lit;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->id()));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    pKF->pose(toAffine(SE3quat).inverse());
  }

  // Points
  for (list<boost::shared_ptr<MapPoint>>::iterator lit = lLocalMapPoints.begin(),
                                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    boost::shared_ptr<MapPoint> pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetPos(vPoint->estimate());
//    pMP->UpdateNormalAndDepth();
  }
}

// void Optimizer::OptimizeEssentialGraph (
// std::shared_ptr<Map> pMap,
// std::shared_ptr<KeyFrame> pLoopKF,
// std::shared_ptr<KeyFrame> pCurKF,
// const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
// const LoopClosing::KeyFrameAndPose& CorrectedSim3,
// const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>&
// LoopConnections,
// const bool& bFixScale)
//{
//    // Setup optimizer
//    g2o::SparseOptimizer optimizer;
//    optimizer.setVerbose (false);
//    g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
//    new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType> ();
//    g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3
//    (linearSolver);
//    g2o::OptimizationAlgorithmLevenberg* solver =
//    new g2o::OptimizationAlgorithmLevenberg (solver_ptr);

//    solver->setUserLambdaInit (1e-16);
//    optimizer.setAlgorithm (solver);

//    const vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames
//    ();
//    const vector<std::shared_ptr<MapPoint>> vpMPs = pMap->GetAllMapPoints
//    ();

//    const unsigned int nMaxKFid = pMap->GetMaxKFid ();

//    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw (nMaxKFid +
//    1);
//    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc
//    (nMaxKFid + 1);
//    vector<g2o::VertexSim3Expmap*> vpVertices (nMaxKFid + 1);

//    const int minFeat = 100;

//    // Set KeyFrame vertices
//    for (size_t i = 0, iend = vpKFs.size (); i < iend; i++)
//    {
//        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
//        if (pKF->isBad ()) continue;
//        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap ();

//        const int nIDi = pKF->mnId;

//        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find
//        (pKF);

//        if (it != CorrectedSim3.end ())
//        {
//            vScw[nIDi] = it->second;
//            VSim3->setEstimate (it->second);
//        }
//        else
//        {
//            Eigen::Matrix<double, 3, 3> Rcw =
//            Converter::toMatrix3d (pKF->GetRotation ());
//            Eigen::Matrix<double, 3, 1> tcw =
//            Converter::toVector3d (pKF->GetTranslation ());
//            g2o::Sim3 Siw (Rcw, tcw, 1.0);
//            vScw[nIDi] = Siw;
//            VSim3->setEstimate (Siw);
//        }

//        if (pKF == pLoopKF) VSim3->setFixed (true);

//        VSim3->setId (nIDi);
//        VSim3->setMarginalized (false);
//        VSim3->_fix_scale = bFixScale;

//        optimizer.addVertex (VSim3);

//        vpVertices[nIDi] = VSim3;
//    }

//    set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

//    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7,
//    7>::Identity ();

//    // Set Loop edges
//    for (map<std::shared_ptr<KeyFrame>,
//    set<std::shared_ptr<KeyFrame>>>::const_iterator
//         mit = LoopConnections.begin (),
//         mend = LoopConnections.end ();
//         mit != mend; mit++)
//    {
//        std::shared_ptr<KeyFrame> pKF = mit->first;
//        const long unsigned int nIDi = pKF->mnId;
//        const set<std::shared_ptr<KeyFrame>>& spConnections = mit->second;
//        const g2o::Sim3 Siw = vScw[nIDi];
//        const g2o::Sim3 Swi = Siw.inverse ();

//        for (set<std::shared_ptr<KeyFrame>>::const_iterator sit =
//        spConnections.begin (),
//                                                            send =
//                                                            spConnections.end
//                                                            ();
//             sit != send; sit++)
//        {
//            const long unsigned int nIDj = (*sit)->mnId;
//            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
//                pKF->GetWeight (*sit) < minFeat)
//                continue;

//            const g2o::Sim3 Sjw = vScw[nIDj];
//            const g2o::Sim3 Sji = Sjw * Swi;

//            g2o::EdgeSim3* e = new g2o::EdgeSim3 ();
//            e->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                             optimizer.vertex (nIDj)));
//            e->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                             optimizer.vertex (nIDi)));
//            e->setMeasurement (Sji);

//            e->information () = matLambda;

//            optimizer.addEdge (e);

//            sInsertedEdges.insert (make_pair (min (nIDi, nIDj), max (nIDi,
//            nIDj)));
//        }
//    }

//    // Set normal edges
//    for (size_t i = 0, iend = vpKFs.size (); i < iend; i++)
//    {
//        std::shared_ptr<KeyFrame> pKF = vpKFs[i];

//        const int nIDi = pKF->mnId;

//        g2o::Sim3 Swi;

//        LoopClosing::KeyFrameAndPose::const_iterator iti =
//        NonCorrectedSim3.find (pKF);

//        if (iti != NonCorrectedSim3.end ())
//            Swi = (iti->second).inverse ();
//        else
//            Swi = vScw[nIDi].inverse ();

//        std::shared_ptr<KeyFrame> pParentKF = pKF->GetParent ();

//        // Spanning tree edge
//        if (pParentKF)
//        {
//            int nIDj = pParentKF->mnId;

//            g2o::Sim3 Sjw;

//            LoopClosing::KeyFrameAndPose::const_iterator itj =
//            NonCorrectedSim3.find (pParentKF);

//            if (itj != NonCorrectedSim3.end ())
//                Sjw = itj->second;
//            else
//                Sjw = vScw[nIDj];

//            g2o::Sim3 Sji = Sjw * Swi;

//            g2o::EdgeSim3* e = new g2o::EdgeSim3 ();
//            e->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                             optimizer.vertex (nIDj)));
//            e->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                             optimizer.vertex (nIDi)));
//            e->setMeasurement (Sji);

//            e->information () = matLambda;
//            optimizer.addEdge (e);
//        }

//        // Loop edges
//        const set<std::shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges
//        ();
//        for (set<std::shared_ptr<KeyFrame>>::const_iterator sit =
//        sLoopEdges.begin (),
//                                                            send =
//                                                            sLoopEdges.end
//                                                            ();
//             sit != send; sit++)
//        {
//            std::shared_ptr<KeyFrame> pLKF = *sit;
//            if (pLKF->mnId < pKF->mnId)
//            {
//                g2o::Sim3 Slw;

//                LoopClosing::KeyFrameAndPose::const_iterator itl =
//                NonCorrectedSim3.find (pLKF);

//                if (itl != NonCorrectedSim3.end ())
//                    Slw = itl->second;
//                else
//                    Slw = vScw[pLKF->mnId];

//                g2o::Sim3 Sli = Slw * Swi;
//                g2o::EdgeSim3* el = new g2o::EdgeSim3 ();
//                el->setVertex (1,
//                dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                                  optimizer.vertex (pLKF->mnId)));
//                el->setVertex (0,
//                dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                                  optimizer.vertex (nIDi)));
//                el->setMeasurement (Sli);
//                el->information () = matLambda;
//                optimizer.addEdge (el);
//            }
//        }

//        // Covisibility graph edges
//        const vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
//        pKF->GetCovisiblesByWeight (minFeat);
//        for (vector<std::shared_ptr<KeyFrame>>::const_iterator vit =
//             vpConnectedKFs.begin ();
//             vit != vpConnectedKFs.end (); vit++)
//        {
//            std::shared_ptr<KeyFrame> pKFn = *vit;
//            if (pKFn && pKFn != pParentKF && !pKF->hasChild (pKFn) &&
//                !sLoopEdges.count (pKFn))
//            {
//                if (!pKFn->isBad () && pKFn->mnId < pKF->mnId)
//                {
//                    if (sInsertedEdges.count (make_pair (min (pKF->mnId,
//                    pKFn->mnId),
//                                                         max (pKF->mnId,
//                                                         pKFn->mnId))))
//                        continue;

//                    g2o::Sim3 Snw;

//                    LoopClosing::KeyFrameAndPose::const_iterator itn =
//                    NonCorrectedSim3.find (pKFn);

//                    if (itn != NonCorrectedSim3.end ())
//                        Snw = itn->second;
//                    else
//                        Snw = vScw[pKFn->mnId];

//                    g2o::Sim3 Sni = Snw * Swi;

//                    g2o::EdgeSim3* en = new g2o::EdgeSim3 ();
//                    en->setVertex (1,
//                    dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                                      optimizer.vertex (pKFn->mnId)));
//                    en->setVertex (0,
//                    dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                                      optimizer.vertex (nIDi)));
//                    en->setMeasurement (Sni);
//                    en->information () = matLambda;
//                    optimizer.addEdge (en);
//                }
//            }
//        }
//    }

//    // Optimize!
//    optimizer.initializeOptimization ();
//    optimizer.optimize (20);

//    unique_lock<mutex> lock (pMap->mMutexMapUpdate);

//    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
//    for (size_t i = 0; i < vpKFs.size (); i++)
//    {
//        std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

//        const int nIDi = pKFi->mnId;

//        g2o::VertexSim3Expmap* VSim3 =
//        static_cast<g2o::VertexSim3Expmap*> (optimizer.vertex (nIDi));
//        g2o::Sim3 CorrectedSiw = VSim3->estimate ();
//        vCorrectedSwc[nIDi] = CorrectedSiw.inverse ();
//        Eigen::Matrix3d eigR = CorrectedSiw.rotation ().toRotationMatrix ();
//        Eigen::Vector3d eigt = CorrectedSiw.translation ();
//        double s = CorrectedSiw.scale ();

//        eigt *= (1. / s); //[R t/s;0 1]

//        cv::Mat Tiw = Converter::toCvSE3 (eigR, eigt);

//        pKFi->SetPose (Tiw);
//    }

//    // Correct points. Transform to "non-optimized" reference keyframe pose
//    and
//    // transform back with optimized pose
//    for (size_t i = 0, iend = vpMPs.size (); i < iend; i++)
//    {
//        std::shared_ptr<MapPoint> pMP = vpMPs[i];

//        if (pMP->isBad ()) continue;

//        int nIDr;
//        if (pMP->mnCorrectedByKF == pCurKF->mnId)
//        {
//            nIDr = pMP->mnCorrectedReference;
//        }
//        else
//        {
//            std::shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame ();
//            nIDr = pRefKF->mnId;
//        }

//        g2o::Sim3 Srw = vScw[nIDr];
//        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

//        cv::Mat P3Dw = pMP->GetWorldPos ();
//        Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d (P3Dw);
//        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
//        correctedSwr.map (Srw.map (eigP3Dw));

//        cv::Mat cvCorrectedP3Dw = Converter::toCvMat (eigCorrectedP3Dw);
//        pMP->SetWorldPos (cvCorrectedP3Dw);

//        pMP->UpdateNormalAndDepth ();
//    }
//}

// int Optimizer::OptimizeSim3 (std::shared_ptr<KeyFrame> pKF1,
//                             std::shared_ptr<KeyFrame> pKF2,
//                             vector<std::shared_ptr<MapPoint>>& vpMatches1,
//                             g2o::Sim3& g2oS12,
//                             const float th2,
//                             const bool bFixScale)
//{
//    g2o::SparseOptimizer optimizer;
//    g2o::BlockSolverX::LinearSolverType* linearSolver;

//    linearSolver = new
//    g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType> ();

//    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX (linearSolver);

//    g2o::OptimizationAlgorithmLevenberg* solver =
//    new g2o::OptimizationAlgorithmLevenberg (solver_ptr);
//    optimizer.setAlgorithm (solver);

//    // Calibration
//    const cv::Mat& K1 = pKF1->mK;
//    const cv::Mat& K2 = pKF2->mK;

//    // Camera poses
//    const cv::Mat R1w = pKF1->GetRotation ();
//    const cv::Mat t1w = pKF1->GetTranslation ();
//    const cv::Mat R2w = pKF2->GetRotation ();
//    const cv::Mat t2w = pKF2->GetTranslation ();

//    // Set Sim3 vertex
//    g2o::VertexSim3Expmap* vSim3 = new g2o::VertexSim3Expmap ();
//    vSim3->_fix_scale = bFixScale;
//    vSim3->setEstimate (g2oS12);
//    vSim3->setId (0);
//    vSim3->setFixed (false);
//    vSim3->_principle_point1[0] = K1.at<float> (0, 2);
//    vSim3->_principle_point1[1] = K1.at<float> (1, 2);
//    vSim3->_focal_length1[0] = K1.at<float> (0, 0);
//    vSim3->_focal_length1[1] = K1.at<float> (1, 1);
//    vSim3->_principle_point2[0] = K2.at<float> (0, 2);
//    vSim3->_principle_point2[1] = K2.at<float> (1, 2);
//    vSim3->_focal_length2[0] = K2.at<float> (0, 0);
//    vSim3->_focal_length2[1] = K2.at<float> (1, 1);
//    optimizer.addVertex (vSim3);

//    // Set MapPoint vertices
//    const int N = vpMatches1.size ();
//    const vector<std::shared_ptr<MapPoint>> vpMapPoints1 =
//    pKF1->GetMapPointMatches ();
//    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
//    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
//    vector<size_t> vnIndexEdge;

//    vnIndexEdge.reserve (2 * N);
//    vpEdges12.reserve (2 * N);
//    vpEdges21.reserve (2 * N);

//    const float deltaHuber = sqrt (th2);

//    int nCorrespondences = 0;

//    for (int i = 0; i < N; i++)
//    {
//        if (!vpMatches1[i]) continue;

//        std::shared_ptr<MapPoint> pMP1 = vpMapPoints1[i];
//        std::shared_ptr<MapPoint> pMP2 = vpMatches1[i];

//        const int id1 = 2 * i + 1;
//        const int id2 = 2 * (i + 1);

//        const int i2 = pMP2->GetIndexInKeyFrame (pKF2);

//        if (pMP1 && pMP2)
//        {
//            if (!pMP1->isBad () && !pMP2->isBad () && i2 >= 0)
//            {
//                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ
//                ();
//                cv::Mat P3D1w = pMP1->GetWorldPos ();
//                cv::Mat P3D1c = R1w * P3D1w + t1w;
//                vPoint1->setEstimate (Converter::toVector3d (P3D1c));
//                vPoint1->setId (id1);
//                vPoint1->setFixed (true);
//                optimizer.addVertex (vPoint1);

//                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ
//                ();
//                cv::Mat P3D2w = pMP2->GetWorldPos ();
//                cv::Mat P3D2c = R2w * P3D2w + t2w;
//                vPoint2->setEstimate (Converter::toVector3d (P3D2c));
//                vPoint2->setId (id2);
//                vPoint2->setFixed (true);
//                optimizer.addVertex (vPoint2);
//            }
//            else
//                continue;
//        }
//        else
//            continue;

//        nCorrespondences++;

//        // Set edge x1 = S12*X2
//        Eigen::Matrix<double, 2, 1> obs1;
//        const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
//        obs1 << kpUn1.pt.x, kpUn1.pt.y;

//        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ ();
//        e12->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                           optimizer.vertex (id2)));
//        e12->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                           optimizer.vertex (0)));
//        e12->setMeasurement (obs1);
//        const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
//        e12->setInformation (Eigen::Matrix2d::Identity () *
//        invSigmaSquare1);

//        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
//        e12->setRobustKernel (rk1);
//        rk1->setDelta (deltaHuber);
//        optimizer.addEdge (e12);

//        // Set edge x2 = S21*X1
//        Eigen::Matrix<double, 2, 1> obs2;
//        const cv::KeyPoint& kpUn2 = pKF2->mvKeysUn[i2];
//        obs2 << kpUn2.pt.x, kpUn2.pt.y;

//        g2o::EdgeInverseSim3ProjectXYZ* e21 = new
//        g2o::EdgeInverseSim3ProjectXYZ ();

//        e21->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                           optimizer.vertex (id1)));
//        e21->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*> (
//                           optimizer.vertex (0)));
//        e21->setMeasurement (obs2);
//        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
//        e21->setInformation (Eigen::Matrix2d::Identity () *
//        invSigmaSquare2);

//        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
//        e21->setRobustKernel (rk2);
//        rk2->setDelta (deltaHuber);
//        optimizer.addEdge (e21);

//        vpEdges12.push_back (e12);
//        vpEdges21.push_back (e21);
//        vnIndexEdge.push_back (i);
//    }

//    // Optimize!
//    optimizer.initializeOptimization ();
//    optimizer.optimize (5);

//    // Check inliers
//    int nBad = 0;
//    for (size_t i = 0; i < vpEdges12.size (); i++)
//    {
//        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
//        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
//        if (!e12 || !e21) continue;

//        if (e12->chi2 () > th2 || e21->chi2 () > th2)
//        {
//            size_t idx = vnIndexEdge[i];
//            //            vpMatches1[idx] = static_cast<MapPoint*> (NULL);
//            vpMatches1[idx].reset ();
//            optimizer.removeEdge (e12);
//            optimizer.removeEdge (e21);
//            vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ*> (NULL);
//            vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ*>
//            (NULL);
//            nBad++;
//        }
//    }

//    int nMoreIterations;
//    if (nBad > 0)
//        nMoreIterations = 10;
//    else
//        nMoreIterations = 5;

//    if (nCorrespondences - nBad < 10) return 0;

//    // Optimize again only with inliers

//    optimizer.initializeOptimization ();
//    optimizer.optimize (nMoreIterations);

//    int nIn = 0;
//    for (size_t i = 0; i < vpEdges12.size (); i++)
//    {
//        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
//        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
//        if (!e12 || !e21) continue;

//        if (e12->chi2 () > th2 || e21->chi2 () > th2)
//        {
//            size_t idx = vnIndexEdge[i];
//            //            vpMatches1[idx] = static_cast<MapPoint*> (NULL);
//            vpMatches1[idx].reset ();
//        }
//        else
//            nIn++;
//    }

//    // Recover optimized Sim3
//    g2o::VertexSim3Expmap* vSim3_recov =
//    static_cast<g2o::VertexSim3Expmap*> (optimizer.vertex (0));
//    g2oS12 = vSim3_recov->estimate ();

//    return nIn;
//}

// int Optimizer::OptimizeSim3FitGPS (std::vector<std::shared_ptr<KeyFrame>>
// &vpKFs,
//                                   std::vector<cv::Point3d> &vGPSPoints,
//                                   g2o::Sim3& g2oSim3,
//                                   const float th2)
//{
//    g2o::SparseOptimizer optimizer;
//    g2o::BlockSolverX::LinearSolverType* linearSolver;

//    linearSolver = new
//    g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType> ();

//    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX (linearSolver);

//    g2o::OptimizationAlgorithmLevenberg* solver =
//    new g2o::OptimizationAlgorithmLevenberg (solver_ptr);
//    optimizer.setAlgorithm (solver);

//    // Set Sim3 vertex
//    g2o::VertexSim3Expmap* vSim3 = new g2o::VertexSim3Expmap ();
//    vSim3->_fix_scale = false;
//    vSim3->setEstimate (g2oSim3);
//    vSim3->setId (0);
//    vSim3->setFixed (false);

//    optimizer.addVertex (vSim3);

//    // Set KeyFrame vertices
//    const int N = vpKFs.size ();
//    vector<g2o::EdgeSim3ProjectXYZGPS*> vpEdgesGPS;
//    vpEdgesGPS.reserve (2 * N);

//    const float deltaHuber = sqrt (th2);

//    int nCorrespondences = 0;

//    for (int i = 0; i < N; i++)
//    {
//        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
//        if (pKF->isBad ()) continue;

//        int id = i + 1;

//        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ ();
//        cv::Mat pose = pKF->GetPoseInverse ();
//        cv::Mat t = pose.rowRange (0, 3).col (3);
//        vPoint->setEstimate (Converter::toVector3d (t));
//        vPoint->setId (id);
//        vPoint->setFixed (false);
//        optimizer.addVertex (vPoint);

//        nCorrespondences++;

//        // Set edge
//        Eigen::Matrix<double, 3, 1> obs;
//        const cv::Point3d& gpsPoint = vGPSPoints[i];
//        obs << gpsPoint.x, gpsPoint.y, gpsPoint.z;

//        g2o::EdgeSim3ProjectXYZGPS* e = new g2o::EdgeSim3ProjectXYZGPS ();
//        e->setVertex (0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//        (optimizer.vertex (id)));
//        e->setVertex (1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
//        (optimizer.vertex (0)));
//        e->setMeasurement (obs);
//        //        const float& invSigmaSquare1 =
//        //        pKF1->mvInvLevelSigma2[kpUn1.octave];
//        //        e12->setInformation (Eigen::Matrix2d::Identity () *
//        //        invSigmaSquare1);
//        e->setInformation (Eigen::Matrix3d::Identity ());

//        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//        e->setRobustKernel (rk);
//        rk->setDelta (deltaHuber);
//        optimizer.addEdge (e);

//        vpEdgesGPS.push_back (e);
//    }

//    // Optimize!
//    optimizer.initializeOptimization ();
//    optimizer.optimize (5);

//    // Check inliers
//    int nBad = 0;
//    for (size_t i = 0; i < vpEdgesGPS.size (); i++)
//    {
//        g2o::EdgeSim3ProjectXYZGPS* e = vpEdgesGPS[i];
//        if (!e) continue;

//        if (e->chi2 () > th2)
//        {
//            optimizer.removeEdge (e);
//            vpEdgesGPS[i] = static_cast<g2o::EdgeSim3ProjectXYZGPS*> (NULL);
//            nBad++;
//        }
//    }
//    LOG(INFO) << "vpEdgesGPS.size = " << vpEdgesGPS.size() << ", nBad = " <<
//    nBad;

//    int nMoreIterations;
//    if (nBad > 0)
//        nMoreIterations = 10;
//    else
//        nMoreIterations = 5;

//    if (nCorrespondences - nBad < 10) return 0;

//    // Optimize again only with inliers

//    optimizer.initializeOptimization ();
//    optimizer.optimize (nMoreIterations);

//    // Recover optimized Sim3
//    g2o::VertexSim3Expmap* vSim3_recov =
//    static_cast<g2o::VertexSim3Expmap*> (optimizer.vertex (0));
//    g2oSim3 = vSim3_recov->estimate ();

//    LOG(INFO) << "g2oSim3.scale = " << g2oSim3.scale();

//    int nIn = 0;
//    for (size_t i = 0; i < vpEdgesGPS.size (); i++)
//    {
//        g2o::EdgeSim3ProjectXYZGPS* e = vpEdgesGPS[i];
//        if (!e) continue;

//        if (e->chi2 () <= th2)
//        {
//            nIn++;

//            //recover estimated pose
//            std::shared_ptr<KeyFrame> pKF = vpKFs[i];
//            cv::Mat Twc = pKF->GetPoseInverse();

//            g2o::VertexSBAPointXYZ *p =
//            static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+1));
//            g2o::Vector3d estimate_t = g2oSim3.map(p->estimate());

//            LOG(INFO) << "Twc.t = " << Twc.at<float>(0, 3) << ", " <<
//            Twc.at<float>(1, 3) << ", " << Twc.at<float>(2, 3)
//                      << ", estimate_t = " << estimate_t[0] << ", " <<
//                      estimate_t[1] << ", " << estimate_t[2];
//            Twc.at<float>(0, 3) = estimate_t[0];
//            Twc.at<float>(1, 3) = estimate_t[1];
//            Twc.at<float>(2, 3) = estimate_t[2];

//            cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
//            cv::Mat twc = Twc.rowRange(0, 3).col(3);

//            cv::Mat Rcw = Rwc.t();
//            cv::Mat tcw = -Rcw*twc;

//            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
//            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
//            tcw.copyTo(Tcw.rowRange(0, 3).col(3));

//            pKF->SetPose(Tcw);
//        }
//    }

//    LOG(INFO) << "vpEdgesGPS.size = " << vpEdgesGPS.size() << ", nIn = " <<
//    nIn;

//    return nIn;
//}

}  // namespace ORB_SLAM
