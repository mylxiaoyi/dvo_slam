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

#include <dvo_slam/keyframe_constraint_search.h>
#include <dvo_slam/keyframe_graph.h>
#include <dvo_slam/local_tracker.h>
#include <dvo_slam/timestamped.h>

#include <dvo_slam/constraints/constraint_proposal.h>
#include <dvo_slam/constraints/constraint_proposal_validator.h>
#include <dvo_slam/constraints/constraint_proposal_voter.h>

#include <dvo_slam/ORBmatcher.h>
#include <dvo_slam/Optimizer.h>
#include <dvo_slam/map.h>
#include <dvo_slam/mappoint.h>
#include <dvo_slam/orbvocabulary.h>

#include <dvo/dense_tracking.h>
#include <dvo/util/stopwatch.h>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_queue.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/mutex.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb_thread.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/core/estimate_propagator.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>

using namespace dvo_slam::constraints;

namespace dvo_slam {

namespace internal {

struct FindEdgeById {
 public:
  FindEdgeById(int id) : id_(id) {}

  bool operator()(g2o::HyperGraph::Edge* e) { return e->id() == id_; }

 private:
  int id_;
};

static Eigen::Isometry3d toIsometry(const Eigen::Affine3d& pose) {
  Eigen::Isometry3d p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

static Eigen::Affine3d toAffine(const Eigen::Isometry3d& pose) {
  Eigen::Affine3d p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

static inline int combine(const short& left, const short& right) {
  int lower, upper;

  if (left < right) {
    lower = right;
    upper = left;
  } else {
    lower = left;
    upper = right;
  }

  return upper << 16 | lower;
}

class KeyframeGraphImpl {
 public:
  friend class ::dvo_slam::KeyframeGraph;
  static const int FirstOdometryId = 1 << 30;
  KeyframeGraphImpl()
      : optimization_thread_shutdown_(false),
        optimization_thread_(
            boost::bind(&KeyframeGraphImpl::execOptimization, this)),
        next_keyframe_id_(1),
        next_odometry_vertex_id_(FirstOdometryId),
        next_odometry_edge_id_(FirstOdometryId),
        validator_pool_(boost::bind(
            &KeyframeGraphImpl::createConstraintProposalValidator, this)) {
    // g2o setup
    keyframegraph_.setAlgorithm(new g2o::OptimizationAlgorithmDogleg(
        new BlockSolver(new LinearSolver())));
    keyframegraph_.setVerbose(false);

    configure(cfg_);

    // mylxiaoyi
    mpMap = boost::make_shared<dvo_slam::Map>();
    mpVocabulary = boost::make_shared<dvo_slam::ORBVocabulary>();
    std::string voc_file =
        "/media/datadisk/work/ros_dvoslam/src/dvo_slam/dvo_slam/voc/ORBvoc.bin";
    if (mpVocabulary->loadFromBinaryFile(voc_file))
      std::cout << "load voc file successfuly" << std::endl;
  }

  ~KeyframeGraphImpl() {
    optimization_thread_shutdown_ = true;
    new_keyframes_.push(LocalMap::Ptr());
    optimization_thread_.join();
  }

  void configure(const dvo_slam::KeyframeGraphConfig& cfg) {
    cfg_ = cfg;
    constraint_search_.reset(
        new NearestNeighborConstraintSearch(cfg_.NewConstraintSearchRadius));
  }

  void add(const LocalMap::Ptr& keyframe) {
    if (cfg_.UseMultiThreading) {
      new_keyframes_.push(keyframe);
    } else {
      // wait until queue is empty
      tbb::mutex::scoped_lock l(queue_empty_sync_);

      newKeyframe(keyframe);
    }
  }

  g2o::OptimizableGraph::Vertex* findNextVertex(
      g2o::OptimizableGraph::Vertex* v) {
    g2o::OptimizableGraph::Vertex *other = 0, *closest = 0;

    ros::Time ts = dynamic_cast<Timestamped*>(v->userData())->timestamp;

    for (g2o::OptimizableGraph::EdgeSet::iterator e_it = v->edges().begin();
         e_it != v->edges().end(); ++e_it) {
      other = dynamic_cast<g2o::OptimizableGraph::Vertex*>(
          (*e_it)->vertex(0) != v ? (*e_it)->vertex(0) : (*e_it)->vertex(1));

      if (dynamic_cast<Timestamped*>(other->userData())->timestamp < ts)
        continue;

      if (closest != 0) {
        if (dynamic_cast<Timestamped*>(other->userData())->timestamp <
            dynamic_cast<Timestamped*>(closest->userData())->timestamp) {
          closest = other;
        }
      } else {
        closest = other;
      }
    }

    return closest;
  }

  bool hasEdge(g2o::OptimizableGraph::Vertex const* v, int edge_id) {
    bool result = false;

    for (g2o::OptimizableGraph::EdgeSet::const_iterator e_it =
             v->edges().begin();
         e_it != v->edges().end() && !result; ++e_it) {
      result = (*e_it)->id() == edge_id;
    }

    return result;
  }

  void finalOptimization() {
    tbb::mutex::scoped_lock l;
    std::cerr << "final optimization, waiting for all keyframes";
    while (!l.try_acquire(queue_empty_sync_)) {
      std::cerr << ".";
      tbb::this_tbb_thread::sleep(tbb::tick_count::interval_t(0.1));
    }
    std::cerr << std::endl;

    std::cerr << keyframes_.size() << " keyframes" << std::endl;

    for (KeyframeVector::iterator it = keyframes_.begin();
         it != keyframes_.end(); ++it) {
      ConstraintProposalVector constraints;
      KeyframeVector constraint_candidates, filtered_constraint_candidates;
      constraint_search_->findPossibleConstraints(keyframes_, *it,
                                                  constraint_candidates);

      for (KeyframeVector::iterator cc_it = constraint_candidates.begin();
           cc_it != constraint_candidates.end(); ++cc_it) {
        int edge_id = combine((*cc_it)->id(), (*it)->id());
        bool exists = std::abs((*cc_it)->id() - (*it)->id()) <=
                      1;  // odometry constraint or self constraint

        exists = exists ||
                 hasEdge(keyframegraph_.vertex((*it)->id()), edge_id) ||
                 hasEdge(keyframegraph_.vertex((*cc_it)->id()), edge_id);

        if (!exists) {
          filtered_constraint_candidates.push_back(*cc_it);
        }
      }

      // validate constraints
      validateKeyframeConstraintsParallel(filtered_constraint_candidates, *it,
                                          constraints);

      // update graph
      insertNewKeyframeConstraints(constraints);
      std::cerr << constraints.size() << " additional constraints" << std::endl;
    }

    // include all edges in the optimization
    if (cfg_.FinalOptimizationUseDenseGraph &&
        !cfg_.OptimizationUseDenseGraph) {
      for (g2o::OptimizableGraph::EdgeSet::iterator e_it =
               keyframegraph_.edges().begin();
           e_it != keyframegraph_.edges().end(); ++e_it) {
        g2o::EdgeSE3* e = (g2o::EdgeSE3*)(*e_it);
        e->setLevel(0);
      }
    }

    std::cerr << "optimizing..." << std::endl;

    keyframegraph_.setVerbose(true);

    int removed = 0, iterations = -1;
    for (int idx = 0; idx < 10 && (iterations != 0 || removed != 0); ++idx) {
      keyframegraph_.initializeOptimization(0);
      iterations =
          keyframegraph_.optimize(cfg_.FinalOptimizationIterations / 10);

      if (cfg_.FinalOptimizationRemoveOutliers) {
        removed = removeOutlierConstraints(
            cfg_.FinalOptimizationOutlierWeightThreshold);
      }
    }

    std::cerr << "done" << std::endl;

    // update keyframe database
    updateKeyframePosesFromGraph();

    map_changed_(*me_);

    ROS_WARN_STREAM(
        "created validation tracker instances: " << validator_pool_.size());

    std::cerr << keyframes_.size() << " keyframes" << std::endl;
  }

  void optimizeInterKeyframePoses() {
    for (KeyframeVector::iterator it = keyframes_.begin();
         it != (keyframes_.end() - 1); ++it) {
      g2o::OptimizableGraph::VertexSet inter_keyframe_vertices;

      g2o::OptimizableGraph::Vertex* v1 = keyframegraph_.vertex((*it)->id());
      g2o::OptimizableGraph::Vertex* v2 =
          keyframegraph_.vertex((*(it + 1))->id());

      v1->setFixed(true);
      v2->setFixed(true);

      inter_keyframe_vertices.insert(v1);
      inter_keyframe_vertices.insert(v2);

      for (g2o::OptimizableGraph::EdgeSet::iterator edge_it =
               v1->edges().begin();
           edge_it != v1->edges().end(); ++edge_it) {
        g2o::HyperGraph::Vertex* candidate = 0;
        if ((*edge_it)->vertex(0)->id() == v1->id()) {
          candidate = (*edge_it)->vertex(1);
        }

        if ((*edge_it)->vertex(1)->id() == v1->id()) {
          candidate = (*edge_it)->vertex(0);
        }

        if (candidate != 0 && candidate->id() >= FirstOdometryId) {
          inter_keyframe_vertices.insert(candidate);
        }
      }

      keyframegraph_.setVerbose(false);
      keyframegraph_.initializeOptimization(inter_keyframe_vertices, 2);
      keyframegraph_.optimize(20);
    }
  }

  cv::Mat computeIntensityErrorImage(int edge_id, bool use_measurement) {
    cv::Mat result;

    g2o::OptimizableGraph::EdgeSet::iterator edge_it =
        std::find_if(keyframegraph_.edges().begin(),
                     keyframegraph_.edges().end(), FindEdgeById(edge_id));

    if (edge_it == keyframegraph_.edges().end()) return result;

    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*edge_it);

    KeyframePtr& kf1 = keyframes_[(*edge_it)->vertex(0)->id() - 1];
    KeyframePtr& kf2 = keyframes_[(*edge_it)->vertex(1)->id() - 1];
    assert(kf1->id() == (*edge_it)->vertex(0)->id());
    assert(kf2->id() == (*edge_it)->vertex(1)->id());

    Eigen::Affine3d transform;
    transform = use_measurement ? toAffine(e->measurement())
                                : kf1->pose().inverse() * kf2->pose();

    dvo::DenseTracker tracker;
    tracker.configure(validation_tracker_cfg_);
    result = tracker.computeIntensityErrorImage(*kf2->image(), *kf1->image(),
                                                transform);

    std::map<int, LocalTracker::TrackingResult>::iterator r =
        constraint_tracking_results_.find(edge_id);

    if (r != constraint_tracking_results_.end()) {
      std::cerr << r->second.Statistics << std::endl;
      std::cerr << "min_entropy_ratio_fine: "
                << std::min(kf1->evaluation()->ratioWithAverage(r->second),
                            kf2->evaluation()->ratioWithAverage(r->second))
                << std::endl;

      Eigen::Vector3d rho;
      if (e->robustKernel() != 0)
        e->robustKernel()->robustify(e->chi2(), rho);
      else
        rho.setOnes();

      std::cerr << "chi2: " << e->chi2() << " weight: " << rho(1) << std::endl;
      std::cerr << "kappa fine: "
                << r->second.Statistics.Levels.back()
                       .LastIterationWithIncrement()
                       .InformationConditionNumber()
                << std::endl;
      std::cerr << "kappa coarse: "
                << r->second.Statistics.Levels.front()
                       .LastIterationWithIncrement()
                       .InformationConditionNumber()
                << std::endl;
    }

    return result;
  }

  void debugLoopClosureConstraint(int keyframe1, int keyframe2) {
    std::cerr << dynamic_cast<NearestNeighborConstraintSearch*>(
                     constraint_search_.get())
                     ->maxDistance()
              << std::endl;

    KeyframePtr kf1 = keyframes_[keyframe1 - 1];
    KeyframePtr kf2 = keyframes_[keyframe2 - 1];

    if (!kf1 || !kf2) return;

    ConstraintProposalVector initial_proposals;
    initial_proposals.push_back(
        ConstraintProposal::createWithIdentity(kf1, kf2));
    initial_proposals.push_back(
        ConstraintProposal::createWithRelative(kf1, kf2));
    // initial_proposals.push_back(initial_proposals[0]->createInverseProposal());
    // initial_proposals.push_back(initial_proposals[1]->createInverseProposal());
    ConstraintProposalValidatorPtr& validator = validator_pool_.local();

    validator->validate(initial_proposals, true);
  }

 private:
  typedef g2o::BlockSolver_6_3 BlockSolver;
  typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;

  typedef tbb::enumerable_thread_specific<ConstraintProposalValidatorPtr>
      ConstraintProposalValidatorPool;

  void execOptimization() {
    static dvo::util::stopwatch sw_nkf("new_kf", 10);

    bool is_locked = false;

    while (!optimization_thread_shutdown_) {
      LocalMap::Ptr new_keyframe;

      new_keyframes_.pop(new_keyframe);

      if (new_keyframe) {
        if (!is_locked) {
          queue_empty_sync_.lock();
          is_locked = true;
        }

        sw_nkf.start();
        newKeyframe(new_keyframe);
        sw_nkf.stopAndPrint();

        if (is_locked && new_keyframes_.empty()) {
          queue_empty_sync_.unlock();
          is_locked = false;
        }
      }
    }
  }

  void newKeyframe(const LocalMap::Ptr& map) {
    tbb::mutex::scoped_lock l(new_keyframe_sync_);

    static dvo::util::stopwatch sw_constraint("constraint_search", 50),
        sw_validation("constraint_validation", 50),
        sw_insert("constraint_insert", 50),
        sw_opt("constraint_optimization", 50);

    KeyframeVector constraint_candidates;
    ConstraintProposalVector constraints;

    // insert keyframe into data structures
    KeyframePtr keyframe = insertNewKeyframe(map);

    // mylxiaoyi: detect features
    keyframe->detectFeatures();
    keyframe->mpORBvocabulary = mpVocabulary;

    // early abort
    if (keyframes_.size() == 1) {
      keyframe->CreateInitMap();
      std::cout << "map points in first keyframe = "
                << keyframe->mvpMapPoints.size() << std::endl;
      return;
    } else if (keyframes_.size() > 1) {
      KeyframePtr last_keyframe = keyframes_[keyframes_.size() - 2];
      ORBmatcher matcher(0.9, true);

      int nmatches =
          matcher.SearchByProjection(keyframe, last_keyframe, 15, false);

      if (nmatches < 20) {
        matcher.SearchByProjection(keyframe, last_keyframe, 30, false);
      }

      std::cout << "nmatches = " << nmatches << std::endl;
      Optimizer::PoseOptimization(keyframe);

      //      keyframe->CreateInitMap();

      // discards outliers
      for (size_t i = 0; i < keyframe->N; i++) {
        boost::shared_ptr<MapPoint> pMP = keyframe->mvpMapPoints[i];
        if (pMP) {
          if (keyframe->mvbOutlier[i]) {
            keyframe->mvpMapPoints[i].reset();
            keyframe->mvbOutlier[i] = false;
          }
        }
      }

      ProcessNewKeyFrame(keyframe);

      // Triangulate new MapPoints
      CreateNewMapPoints(keyframe);

      //      bool mbAbortBA = false;
      //      Optimizer::LocalBundleAdjustment (keyframe, &mbAbortBA, mpMap);
    }

    map_changed_(*me_);
  }

  void ProcessNewKeyFrame(KeyframePtr keyframe) {
    // Compute Bags of Words structures
    keyframe->ComputeBoW();

    for (size_t i = 0; i < keyframe->mvpMapPoints.size(); i++) {
      boost::shared_ptr<MapPoint> pMP = keyframe->mvpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          if (!pMP->IsInKeyFrame(keyframe)) {
            pMP->AddObservation(keyframe, i);
            //            pMP->UpdateNormalAndDepth();
            pMP->ComputeDistinctiveDescriptors();
          }
        }
      }
    }

    // Update links in the Covisibility Graph
    keyframe->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(keyframe);
  }

  void CreateNewMapPoints(KeyframePtr keyframe) {
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    const std::vector<boost::shared_ptr<Keyframe>> vpNeighKFs =
        keyframe->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6, false);

    //    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    //    cv::Mat Rwc1 = Rcw1.t();
    //    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    //    cv::Mat Tcw1(3, 4, CV_32F);
    //    Rcw1.copyTo(Tcw1.colRange(0, 3));
    //    tcw1.copyTo(Tcw1.col(3));
    //    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    Eigen::Affine3d pose1_wc = keyframe->pose();
    Eigen::Affine3d pose1_cw = pose1_wc.inverse();
    Eigen::Matrix<double, 3, 3> Rwc1 = pose1_wc.rotation();
    Eigen::Vector3d Ow1 = -pose1_wc.rotation() * pose1_cw.translation();
    Eigen::Matrix<double, 3, 3> Rcw1 = pose1_cw.rotation();
    Eigen::Vector3d tcw1 = pose1_cw.translation();
    Eigen::Matrix<double, 3, 4> Tcw1;
    Tcw1 << Rcw1(0, 0), Rcw1(0, 1), Rcw1(0, 2), tcw1(0),
            Rcw1(1, 0), Rcw1(1, 1), Rcw1(1, 2), tcw1(1),
            Rcw1(2, 0), Rcw1(2, 1), Rcw1(2, 2), tcw1(2);

    const float& fx1 = keyframe->fx;
    const float& fy1 = keyframe->fy;
    const float& cx1 = keyframe->cx;
    const float& cy1 = keyframe->cy;
    const float& invfx1 = 1.0 / fx1;
    const float& invfy1 = 1.0 / fy1;

    const float ratioFactor = 1.5f * keyframe->mfScaleFactor;

    int nnew = 0;

    // Search matches with epipolar restriction and triangulate
    for (size_t i = 0; i < vpNeighKFs.size(); i++) {
      //      if (i > 0 && CheckNewKeyFrames()) return;

      boost::shared_ptr<Keyframe> pKF2 = vpNeighKFs[i];

      // Check first that baseline is not too short
      //      cv::Mat Ow2 = pKF2->GetCameraCenter();
      //      cv::Mat vBaseline = Ow2 - Ow1;
      //      const float baseline = cv::norm(vBaseline);
      Eigen::Affine3d pose2_wc = pKF2->pose();
      Eigen::Affine3d pose2_cw = pose2_wc.inverse();
      Eigen::Matrix<double, 3, 3> Rwc2 = pose2_wc.rotation();
      Eigen::Vector3d Ow2 = -pose2_wc.rotation() * pose2_cw.translation();
      Eigen::Vector3d vBaseline = Ow2 - Ow1;
      const float baseline = vBaseline.norm();
      if (baseline < pKF2->mb) continue;

      // Compute Fundamental Matrix
      Eigen::Matrix<double, 3, 3> F12 = ComputeF12(keyframe, pKF2);

      // Search matches that fullfil epipolar constraint
      std::vector<std::pair<size_t, size_t>> vMatchedIndices;
      matcher.SearchForTriangulation(keyframe, pKF2, F12, vMatchedIndices,
                                     false);

      //      cv::Mat Rcw2 = pKF2->GetRotation();
      //      cv::Mat Rwc2 = Rcw2.t();
      //      cv::Mat tcw2 = pKF2->GetTranslation();
      //      cv::Mat Tcw2(3, 4, CV_32F);
      //      Rcw2.copyTo(Tcw2.colRange(0, 3));
      //      tcw2.copyTo(Tcw2.col(3));
      Eigen::Matrix<double, 3, 3> Rcw2 = pose2_cw.rotation();
      Eigen::Vector3d tcw2 = pose2_cw.translation();
      Eigen::Matrix<double, 3, 4> Tcw2;
      Tcw2.col(0) = Rcw2.col(0);
      Tcw2.col(1) = Rcw2.col(1);
      Tcw2.col(2) = Rcw2.col(2);
      Tcw2.col(3) = tcw2;

      const float& fx2 = pKF2->fx;
      const float& fy2 = pKF2->fy;
      const float& cx2 = pKF2->cx;
      const float& cy2 = pKF2->cy;
      const float& invfx2 = 1.0 / fx2;
      const float& invfy2 = 1.0 / fy2;

      // Triangulate each match
      const int nmatches = vMatchedIndices.size();
      for (int ikp = 0; ikp < nmatches; ikp++) {
        const int& idx1 = vMatchedIndices[ikp].first;
        const int& idx2 = vMatchedIndices[ikp].second;

        const cv::KeyPoint& kp1 = keyframe->mvKeys[idx1];
        const float kp1_ur = keyframe->mvuRight[idx1];
        bool bStereo1 = kp1_ur >= 0;

        const cv::KeyPoint& kp2 = pKF2->mvKeys[idx2];
        const float kp2_ur = pKF2->mvuRight[idx2];
        bool bStereo2 = kp2_ur >= 0;

        // Check parallax between rays
        //        cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) *
        //        invfx1,
        //                       (kp1.pt.y - cy1) * invfy1, 1.0);
        //        cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) *
        //        invfx2,
        //                       (kp2.pt.y - cy2) * invfy2, 1.0);

        Eigen::Vector3d xn1, xn2;
        xn1 << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0;
        xn2 << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0;

        Eigen::Vector3d ray1 = Rwc1 * xn1;
        Eigen::Vector3d ray2 = Rwc2 * xn2;
        float tmp1 = ray1.transpose() * ray2;
        float tmp2 = ray1.norm() * ray2.norm();
        const double cosParallaxRays =
            (tmp1) / (tmp2);

        float cosParallaxStereo = cosParallaxRays + 1;
        float cosParallaxStereo1 = cosParallaxStereo;
        float cosParallaxStereo2 = cosParallaxStereo;

        if (bStereo1)
          cosParallaxStereo1 = cos(2 * atan2(keyframe->mb / 2,
                                             keyframe->mvDepth[idx1]));
        else if (bStereo2)
          cosParallaxStereo2 =
              cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

        cosParallaxStereo = std::min(cosParallaxStereo1, cosParallaxStereo2);

        Eigen::Vector3d x3D;
        if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
            (bStereo1 || bStereo2 || cosParallaxRays < 0.9998)) {
          // Linear Triangulation Method
//          cv::Mat A(4, 4, CV_32F);
//          A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
//          A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
//          A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
//          A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

          Eigen::Matrix<double, 4, 4> A;
          A.row(0) = xn1[0] * Tcw1.row(2) - Tcw1.row(0);
          A.row(1) = xn1[1] * Tcw1.row(2) - Tcw1.row(1);
          A.row(2) = xn2[0] * Tcw2.row(2) - Tcw2.row(0);
          A.row(3) = xn2[1] * Tcw2.row(2) - Tcw2.row(1);

//          cv::Mat w, u, vt;
//          cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
          Eigen::Matrix<double, 4, 4> v = svd.matrixV();

          Eigen::Vector4d x3D_tmp = v.col(3);

          if (x3D_tmp[3] == 0) continue;

          x3D = Eigen::Vector3d(x3D_tmp[0], x3D_tmp[1], x3D_tmp[2]) / x3D_tmp[3];

//          x3D = vt.row(3).t();

//          if (x3D.at<float>(3) == 0) continue;

//          // Euclidean coordinates
//          x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
        } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
          x3D = keyframe->UnprojectStereo(idx1);
        } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
          x3D = pKF2->UnprojectStereo(idx2);
        } else
          continue;  // No stereo and very low parallax

//        cv::Mat x3Dt = x3D.t();

        // Check triangulation in front of cameras
        float z1 = Rcw1.row(2) * x3D + tcw1[2];
        if (z1 <= 0) continue;

        float z2 = Rcw2.row(2) * x3D + tcw2[2];
        if (z2 <= 0) continue;

        // Check reprojection error in first keyframe
        const float& sigmaSquare1 =
            keyframe->mvLevelSigma2[kp1.octave];
        const float x1 = Rcw1.row(0) * x3D + tcw1[0];
        const float y1 = Rcw1.row(1) * x3D + tcw1[1];
        const float invz1 = 1.0 / z1;

        if (!bStereo1) {
          float u1 = fx1 * x1 * invz1 + cx1;
          float v1 = fy1 * y1 * invz1 + cy1;
          float errX1 = u1 - kp1.pt.x;
          float errY1 = v1 - kp1.pt.y;
          if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1) continue;
        } else {
          float u1 = fx1 * x1 * invz1 + cx1;
          float u1_r = u1 - keyframe->mbf * invz1;
          float v1 = fy1 * y1 * invz1 + cy1;
          float errX1 = u1 - kp1.pt.x;
          float errY1 = v1 - kp1.pt.y;
          float errX1_r = u1_r - kp1_ur;
          if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
              7.8 * sigmaSquare1)
            continue;
        }

        // Check reprojection error in second keyframe
        const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
        const float x2 = Rcw2.row(0) * x3D + tcw2[0];
        const float y2 = Rcw2.row(1) * x3D + tcw2[1];
        const float invz2 = 1.0 / z2;
        if (!bStereo2) {
          float u2 = fx2 * x2 * invz2 + cx2;
          float v2 = fy2 * y2 * invz2 + cy2;
          float errX2 = u2 - kp2.pt.x;
          float errY2 = v2 - kp2.pt.y;
          if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2) continue;
        } else {
          float u2 = fx2 * x2 * invz2 + cx2;
          float u2_r = u2 - keyframe->mbf * invz2;
          float v2 = fy2 * y2 * invz2 + cy2;
          float errX2 = u2 - kp2.pt.x;
          float errY2 = v2 - kp2.pt.y;
          float errX2_r = u2_r - kp2_ur;
          if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
              7.8 * sigmaSquare2)
            continue;
        }

        // Check scale consistency
//        cv::Mat normal1 = x3D - Ow1;
//        float dist1 = cv::norm(normal1);

//        cv::Mat normal2 = x3D - Ow2;
//        float dist2 = cv::norm(normal2);
        Eigen::Vector3d normal1 = x3D - Ow1;
        float dist1 = normal1.norm();

        Eigen::Vector3d normal2 = x3D - Ow2;
        float dist2 = normal2.norm();

        if (dist1 == 0 || dist2 == 0) continue;

        const float ratioDist = dist2 / dist1;
        const float ratioOctave = keyframe->mvScaleFactors[kp1.octave] /
                                  pKF2->mvScaleFactors[kp2.octave];

        /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
            continue;*/
        if (ratioDist * ratioFactor < ratioOctave ||
            ratioDist > ratioOctave * ratioFactor)
          continue;

        // Triangulation is succesfull
        boost::shared_ptr<MapPoint> pMP(
            new MapPoint(x3D, keyframe /*, mpMap*/));

        pMP->AddObservation(keyframe, idx1);
        pMP->AddObservation(pKF2, idx2);

        keyframe->AddMapPoint(pMP, idx1);
        pKF2->AddMapPoint(pMP, idx2);

        pMP->ComputeDistinctiveDescriptors();

        //        pMP->UpdateNormalAndDepth();

        mpMap->AddMapPoint(pMP);

        nnew++;
      }
    }
  }

  Eigen::Matrix<double, 3, 3> ComputeF12(boost::shared_ptr<Keyframe> pKF1,
                                         boost::shared_ptr<Keyframe> pKF2) {
    //    cv::Mat R1w = pKF1->GetRotation();
    //    cv::Mat t1w = pKF1->GetTranslation();
    //    cv::Mat R2w = pKF2->GetRotation();
    //    cv::Mat t2w = pKF2->GetTranslation();
    Eigen::Affine3d pose1_wc = pKF1->pose();
    Eigen::Affine3d pose1_cw = pose1_wc.inverse();
    Eigen::Matrix<double, 3, 3> R1w = pose1_cw.rotation();
    Eigen::Vector3d t1w = pose1_cw.translation();

    Eigen::Affine3d pose2_wc = pKF2->pose();
    Eigen::Affine3d pose2_cw = pose2_wc.inverse();
    Eigen::Matrix<double, 3, 3> R2w = pose2_cw.rotation();
    Eigen::Vector3d t2w = pose2_cw.translation();

    //    cv::Mat R12 = R1w * R2w.t();
    //    cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;
    Eigen::Matrix<double, 3, 3> R12 = R1w * R2w.transpose();
    Eigen::Vector3d t12 = -R1w * R2w.transpose() * t2w + t1w;

    Eigen::Matrix<double, 3, 3> t12x = SkewSymmetricMatrix(t12);

    //    const cv::Mat& K1 = pKF1->mK;
    //    const cv::Mat& K2 = pKF2->mK;
    Eigen::Matrix<double, 3, 3> K1;
    K1 << pKF1->fx, 0, pKF1->cx, 0, pKF1->fy, pKF1->cy, 0, 0, 1;
    Eigen::Matrix<double, 3, 3> K2;
    K2 << pKF2->fx, 0, pKF2->cx, 0, pKF1->fy, pKF1->cy, 0, 0, 1;

    //    return K1.t().inv() * t12x * R12 * K2.inv();
    return K1.transpose().inverse() * t12x * R12 * K2.inverse();
  }

  Eigen::Matrix<double, 3, 3> SkewSymmetricMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix<double, 3, 3> m;

    m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return m;
  }

  ConstraintProposalValidatorPtr createConstraintProposalValidator() {
    ConstraintProposalValidatorPtr r =
        boost::make_shared<ConstraintProposalValidator>();

    r->createStage(1)
        .trackingConfig(validation_tracker_cfg_)
        .keepAll()
        .addVoter(new OdometryConstraintVoter())
        .addVoter(new NaNResultVoter())
        .addVoter(
            new ConstraintRatioVoter(cfg_.MinEquationSystemConstraintRatio))
        .addVoter(new TrackingResultEvaluationVoter(
            cfg_.NewConstraintMinEntropyRatioCoarse))
        .addVoter(new CrossValidationVoter(1.0));

    r->createStage(2)
        .trackingConfig(constraint_tracker_cfg_)
        .keepBest()
        .addVoter(new NaNResultVoter())
        .addVoter(
            new ConstraintRatioVoter(cfg_.MinEquationSystemConstraintRatio))
        .addVoter(new TrackingResultEvaluationVoter(
            cfg_.NewConstraintMinEntropyRatioFine));

    return r;
  }

  struct ValidateConstraintProposalReduction {
   private:
    ConstraintProposalValidatorPool& validators_;
    ConstraintProposalVector proposals_;

    ConstraintProposalValidatorPtr& validator() { return validators_.local(); }

    void appendProposals(ConstraintProposalVector& pv) {
      proposals_.reserve(proposals_.size() + pv.size());
      proposals_.insert(proposals_.end(), pv.begin(), pv.end());
    }

   public:
    typedef tbb::blocked_range<ConstraintProposalVector::const_iterator>
        ConstraintPoposalConstRange;

    ValidateConstraintProposalReduction(
        ConstraintProposalValidatorPool& validators)
        : validators_(validators) {}

    ValidateConstraintProposalReduction(
        const ValidateConstraintProposalReduction& other, tbb::split)
        : validators_(other.validators_) {}

    ConstraintProposalVector& proposals() { return proposals_; }

    void operator()(const ConstraintPoposalConstRange& r) {
      ConstraintProposalVector proposals;
      proposals.assign(r.begin(), r.end());

      validator()->validate(proposals);

      appendProposals(proposals);
    }

    void join(ValidateConstraintProposalReduction& other) {
      appendProposals(other.proposals_);
      validator()->keepBest(proposals_);
    }
  };

  void validateKeyframeConstraintsParallel(
      const KeyframeVector& constraint_candidates, const KeyframePtr& keyframe,
      ConstraintProposalVector& proposals) {
    ConstraintProposalVector initial_proposals;
    initial_proposals.reserve(constraint_candidates.size() * 2);

    for (KeyframeVector::const_iterator it = constraint_candidates.begin();
         it != constraint_candidates.end(); ++it) {
      initial_proposals.push_back(
          ConstraintProposal::createWithIdentity(keyframe, *it));
      initial_proposals.push_back(
          ConstraintProposal::createWithRelative(keyframe, *it));
    }

    ValidateConstraintProposalReduction body(validator_pool_);
    size_t grain_size = cfg_.UseMultiThreading ? 1 : initial_proposals.size();

    tbb::parallel_reduce(
        ValidateConstraintProposalReduction::ConstraintPoposalConstRange(
            initial_proposals.begin(), initial_proposals.end(), grain_size),
        body);

    proposals = body.proposals();
  }

  int insertNewKeyframeConstraints(const ConstraintProposalVector& proposals) {
    int inserted = 0;
    int max_distance = -1;

    for (ConstraintProposalVector::const_iterator it = proposals.begin();
         it != proposals.end(); ++it) {
      const ConstraintProposalPtr& p = *it;

      int distance = std::abs(p->Reference->id() - p->Current->id());
      bool odometry_constraint = std::abs(distance) == 1;

      assert(!odometry_constraint);

      inserted++;
      insertConstraint(p->Reference, p->Current, p->TrackingResult);

      max_distance = std::max(max_distance, distance);
    }

    return max_distance;
  }

  std::map<int, LocalTracker::TrackingResult> constraint_tracking_results_;

  void insertConstraint(const KeyframePtr& keyframe,
                        const KeyframePtr& constraint,
                        const LocalTracker::TrackingResult& result) {
    int edge_id = combine(constraint->id(), keyframe->id());

    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    e->setId(edge_id);
    e->setMeasurement(toIsometry(result.Transformation));
    e->setRobustKernel(createRobustKernel());
    e->setInformation(result.Information);
    e->resize(2);
    e->setVertex(0, keyframegraph_.vertex(keyframe->id()));
    e->setVertex(1, keyframegraph_.vertex(constraint->id()));

    constraint_tracking_results_[edge_id] = result;

    keyframegraph_.addEdge(e);
  }

  bool isOdometryConstraint(g2o::EdgeSE3* e) {
    return std::abs(e->vertex(0)->id() - e->vertex(1)->id()) == 1;
  }

  int removeOutlierConstraints(double weight_threshold, int n_max = -1) {
    int n_removed = 0;

    std::map<double, g2o::EdgeSE3*> candidate_edges;

    for (g2o::HyperGraph::EdgeSet::iterator it = keyframegraph_.edges().begin();
         it != keyframegraph_.edges().end(); ++it) {
      g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*it);

      if (e->robustKernel() == 0) continue;

      Eigen::Vector3d rho;
      e->robustKernel()->robustify(e->chi2(), rho);

      if (rho[1] < weight_threshold) {
        candidate_edges.insert(
            std::map<double, g2o::EdgeSE3*>::value_type(rho[1], e));
      }
    }

    for (std::map<double, g2o::EdgeSE3*>::iterator it = candidate_edges.begin();
         it != candidate_edges.end() && (n_removed < n_max || n_max < 0);
         ++it) {
      keyframegraph_.removeEdge(it->second);
      ++n_removed;
    }

    if (n_removed > 0)
      std::cerr << "removed: " << n_removed << " edges" << std::endl;

    return n_removed;
  }

  void updateKeyframePosesFromGraph() {
    for (KeyframeVector::iterator it = keyframes_.begin();
         it != keyframes_.end(); ++it) {
      const KeyframePtr& keyframe = *it;

      g2o::VertexSE3* vertex =
          (g2o::VertexSE3*)keyframegraph_.vertex(keyframe->id());

      keyframe->pose(toAffine(vertex->estimate()));
    }
  }

  struct FindEdge {
    int id1, id2;

    FindEdge(int id1, int id2) : id1(id1), id2(id2) {}

    bool operator()(const g2o::HyperGraph::Edge* e) const {
      return e->vertices().size() == 2 &&
             ((e->vertex(0)->id() == id1 && e->vertex(1)->id() == id2) ||
              (e->vertex(1)->id() == id1 && e->vertex(0)->id() == id2));
    }
  };

  void addGraph(g2o::OptimizableGraph* g) {
    for (g2o::HyperGraph::VertexIDMap::iterator it = g->vertices().begin();
         it != g->vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex* v =
          (g2o::OptimizableGraph::Vertex*)(it->second);
      if (keyframegraph_.vertex(v->id())) continue;

      g2o::VertexSE3* v1 = (g2o::VertexSE3*)v;
      g2o::VertexSE3* v2 = new g2o::VertexSE3();
      v2->setId(v1->id());
      v2->setEstimate(v1->estimate());
      v2->setMarginalized(v1->marginalized());
      v2->setUserData(v1->userData());
      v1->setUserData(0);
      // v2->edges().clear();
      v2->setHessianIndex(-1);

      if (!keyframegraph_.addVertex(v2)) {
        throw std::runtime_error("failed to add vertex to g2o graph!");
      }
    }
    for (g2o::HyperGraph::EdgeSet::iterator it = g->edges().begin();
         it != g->edges().end(); ++it) {
      g2o::EdgeSE3* e = (g2o::EdgeSE3*)(*it);
      g2o::EdgeSE3* en = new g2o::EdgeSE3();

      en->setId(e->id());
      en->setLevel(e->level());
      // en->setRobustKernel(createRobustKernel());
      en->setMeasurement(e->measurement());
      en->setInformation(e->information());
      en->resize(e->vertices().size());
      int cnt = 0;
      for (std::vector<g2o::HyperGraph::Vertex*>::const_iterator it =
               e->vertices().begin();
           it != e->vertices().end(); ++it) {
        g2o::OptimizableGraph::Vertex* v =
            (g2o::OptimizableGraph::Vertex*)keyframegraph_.vertex((*it)->id());
        assert(v);
        en->setVertex(cnt++, v);
      }
      keyframegraph_.addEdge(en);
    }
  }

  KeyframePtr insertNewKeyframe(const LocalMap::Ptr& m) {
    // update keyframe pose, because its probably different from the one, which
    // was used during local map initialization
    if (!keyframes_.empty()) {
      g2o::HyperGraph::Vertex* last_kv_tmp =
          keyframegraph_.vertex(next_keyframe_id_ - 1);

      if (last_kv_tmp == 0) {
        throw std::runtime_error("last_kv_tmp == nullptr");
      }

      g2o::VertexSE3* last_kv = dynamic_cast<g2o::VertexSE3*>(last_kv_tmp);
      g2o::OptimizableGraph::EdgeSet::iterator e = std::find_if(
          last_kv->edges().begin(), last_kv->edges().end(),
          FindEdge(next_keyframe_id_ - 1, next_odometry_vertex_id_));

      assert(e != last_kv->edges().end());

      g2o::EdgeSE3* e_se3 = (g2o::EdgeSE3*)(*e);

      m->setKeyframePose(toAffine(last_kv->estimate() * e_se3->measurement()));
    }

    m->optimize();
    g2o::SparseOptimizer& g = m->getGraph();

    int max_id = g.vertices().size();

    g2o::OptimizableGraph::VertexIDMap vertices = g.vertices();
    for (g2o::OptimizableGraph::VertexIDMap::iterator v_it = vertices.begin();
         v_it != vertices.end(); ++v_it) {
      g.changeId(v_it->second,
                 next_odometry_vertex_id_ + (v_it->second->id() - 1));
    }

    for (g2o::OptimizableGraph::EdgeSet::iterator e_it = g.edges().begin();
         e_it != g.edges().end(); ++e_it) {
      g2o::EdgeSE3* e = (g2o::EdgeSE3*)(*e_it);
      e->setId(next_odometry_edge_id_++);
      e->setLevel(cfg_.OptimizationUseDenseGraph ? 0 : 2);
    }

    addGraph(&g);

    // get last odometry vertex from global graph, which will become new
    // keyframe vertex
    g2o::VertexSE3* kv =
        (g2o::VertexSE3*)keyframegraph_.vertex(next_odometry_vertex_id_);
    if (kv == 0) {
      throw std::runtime_error("kv == nullptr");
    }
    if (!keyframegraph_.changeId(kv, next_keyframe_id_)) {
      throw std::runtime_error(
          "keyframegraph_.changeId(kv, next_keyframe_id_) failed!");
    }

    if (!keyframes_.empty()) {
      g2o::VertexSE3* kv =
          (g2o::VertexSE3*)keyframegraph_.vertex(next_keyframe_id_);

      // find the odometry edge, which connects the old keyframe vertex with the
      // new keyframe vertex
      g2o::OptimizableGraph::EdgeSet::iterator ke =
          std::find_if(kv->edges().begin(), kv->edges().end(),
                       FindEdge(next_keyframe_id_ - 1, next_keyframe_id_));

      assert(ke != kv->edges().end());

      // promote odometry edge to keyframe edge
      g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*ke);
      e->setId(combine(next_keyframe_id_ - 1, next_keyframe_id_));
      e->setLevel(0);
    } else {
      kv->setFixed(true);
    }

    // create keyframe
    KeyframePtr keyframe(new Keyframe());
    keyframe->id(next_keyframe_id_)
        .image(m->getKeyframe())
        .pose(toAffine(kv->estimate()))
        .evaluation(m->getEvaluation());

    kv->setUserData(new dvo_slam::Timestamped(keyframe->timestamp()));

    keyframes_.push_back(keyframe);

    // increment ids
    next_odometry_vertex_id_ += max_id - 1;
    next_keyframe_id_ += 1;

    return keyframe;
  }

  void configureValidationTracking(const dvo::DenseTracker::Config& cfg) {
    constraint_tracker_cfg_ = dvo::DenseTracker::getDefaultConfig();
    constraint_tracker_cfg_.FirstLevel = 3;
    constraint_tracker_cfg_.LastLevel = 1;
    constraint_tracker_cfg_.Precision = cfg.Precision;
    constraint_tracker_cfg_.UseInitialEstimate = true;
    constraint_tracker_cfg_.Mu = cfg.Mu;
    constraint_tracker_cfg_.IntensityDerivativeThreshold =
        cfg.IntensityDerivativeThreshold;
    constraint_tracker_cfg_.DepthDerivativeThreshold =
        cfg.DepthDerivativeThreshold;

    validation_tracker_cfg_ = dvo::DenseTracker::getDefaultConfig();
    validation_tracker_cfg_.FirstLevel = 3;
    validation_tracker_cfg_.LastLevel = 3;
    validation_tracker_cfg_.Precision = cfg.Precision;
    validation_tracker_cfg_.UseInitialEstimate = true;
    validation_tracker_cfg_.Mu = cfg.Mu;
    validation_tracker_cfg_.IntensityDerivativeThreshold =
        cfg.IntensityDerivativeThreshold;
    validation_tracker_cfg_.DepthDerivativeThreshold =
        cfg.DepthDerivativeThreshold;
  }

  g2o::RobustKernel* createRobustKernel() {
    if (cfg_.UseRobustKernel) {
      g2o::RobustKernel* k = new g2o::RobustKernelCauchy();
      k->setDelta(5);

      return k;
    } else {
      return 0;
    }
  }

  KeyframePtr active_;
  tbb::concurrent_bounded_queue<LocalMap::Ptr> new_keyframes_;
  bool optimization_thread_shutdown_;
  tbb::tbb_thread optimization_thread_;
  tbb::mutex new_keyframe_sync_, queue_empty_sync_;
  KeyframeConstraintSearchInterfacePtr constraint_search_;
  KeyframeVector keyframes_;
  short next_keyframe_id_;
  int next_odometry_vertex_id_, next_odometry_edge_id_;

  g2o::SparseOptimizer keyframegraph_;
  dvo::DenseTracker::Config validation_tracker_cfg_, constraint_tracker_cfg_;

  dvo_slam::KeyframeGraphConfig cfg_;

  ConstraintProposalValidatorPool validator_pool_;

  KeyframeGraph* me_;

  boost::signals2::signal<KeyframeGraph::MapChangedCallbackSignature>
      map_changed_;

  // mylxiaoyi
  boost::shared_ptr<Map> mpMap;
  boost::shared_ptr<dvo_slam::ORBVocabulary> mpVocabulary;
};
} /* namespace internal */

KeyframeGraph::KeyframeGraph() : impl_(new internal::KeyframeGraphImpl()) {
  impl_->me_ = this;
}

KeyframeGraph::~KeyframeGraph() {}

const dvo_slam::KeyframeGraphConfig& KeyframeGraph::configuration() const {
  return impl_->cfg_;
}

void KeyframeGraph::configure(const KeyframeGraphConfig& config) {
  impl_->configure(config);
}

void KeyframeGraph::configureValidationTracking(
    const dvo::DenseTracker::Config& cfg) {
  impl_->configureValidationTracking(cfg);
}

void KeyframeGraph::add(const LocalMap::Ptr& keyframe) { impl_->add(keyframe); }

void KeyframeGraph::finalOptimization() { impl_->finalOptimization(); }

void KeyframeGraph::addMapChangedCallback(
    const KeyframeGraph::MapChangedCallback& callback) {
  impl_->map_changed_.connect(callback);
}

cv::Mat KeyframeGraph::computeIntensityErrorImage(int edge_id,
                                                  bool use_measurement) const {
  return impl_->computeIntensityErrorImage(edge_id, use_measurement);
}

void KeyframeGraph::debugLoopClosureConstraint(int keyframe1,
                                               int keyframe2) const {
  impl_->debugLoopClosureConstraint(keyframe1, keyframe2);
}

const g2o::SparseOptimizer& KeyframeGraph::graph() const {
  return impl_->keyframegraph_;
}

const KeyframeVector& KeyframeGraph::keyframes() const {
  return impl_->keyframes_;
}

} /* namespace dvo_slam */
