/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsOdometryRejectedMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const GlobalLocalizationOdometry& odometry_check) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() ABSL_LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      odometry_check, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const GlobalLocalizationOdometry& odometry_check) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() ABSL_LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      odometry_check, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    const GlobalLocalizationOdometry& odometry_check,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  }
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Stage 1: odometry-aware gate for loop-closure constraints.
  //
  // Before a match is refined by Ceres and turned into an INTER_SUBMAP
  // constraint, verify that the motion it implies is consistent with trusted
  // odometry (or, for the frozen-map direction that has no odometry, with the
  // assumption that the localization is already correct). This rejects
  // geometrically good but incorrect matches in repetitive environments before
  // they can contaminate the pose graph. It covers full-submap (global) matches
  // AND cross-trajectory local matches (loop closures that land in the
  // "recently connected" local-search window); same-trajectory local
  // refinements are left untouched.
  const bool cross_trajectory =
      node_id.trajectory_id != submap_id.trajectory_id;
  // The gate only fires once the localization has actually been established
  // (the trajectories tied by a real loop closure). Before that the pose-graph
  // estimate is the unlocalized initial guess, so any correct relocalization
  // implies a large correction and must be allowed through.
  if (odometry_check.localization_established &&
      (match_full_submap || cross_trajectory) &&
      options_.global_localization_odometry_check_enabled()) {
    if (odometry_check.has_odometry) {
      // The node's own trajectory has odometry: verify the motion this match
      // implies between the previous and current node is consistent with it.
      const transform::Rigid2d global_relative_motion =
          odometry_check.previous_global_pose_2d.inverse() * pose_estimate;
      const double translation_error =
          (global_relative_motion.translation() -
           odometry_check.relative_odometry_2d.translation())
              .norm();
      const double rotation_error = std::abs(common::NormalizeAngleDifference(
          global_relative_motion.rotation().angle() -
          odometry_check.relative_odometry_2d.rotation().angle()));
      const bool odometry_inconsistent =
          translation_error >
              options_.global_localization_max_odometry_translation_error() ||
          rotation_error >
              options_.global_localization_max_odometry_rotation_error();
      if (odometry_inconsistent) {
        kGlobalConstraintsOdometryRejectedMetric->Increment();
      }
      // Always emit the gate decision at INFO level so both ACCEPT and REJECT
      // are visible in /rosout (and via carto_loger.py) without requiring
      // GLOG_v=1 or log_matches.
      std::ostringstream info;
      info << "[GlobalLoc] node: " << node_id << " submap: " << submap_id
           << " mode=" << (match_full_submap ? "GLOBAL" : "LOCAL-cross")
           << std::fixed << " scan_score: " << std::setprecision(4) << score
           << " odom_translation: " << std::setprecision(4)
           << odometry_check.relative_odometry_2d.translation().norm()
           << " odom_rotation: " << std::setprecision(4)
           << odometry_check.relative_odometry_2d.rotation().angle()
           << " global_relative_translation: " << std::setprecision(4)
           << global_relative_motion.translation().norm()
           << " global_relative_rotation: " << std::setprecision(4)
           << global_relative_motion.rotation().angle()
           << " translation_error: " << std::setprecision(4)
           << translation_error << " rotation_error: " << std::setprecision(4)
           << rotation_error
           << " decision: " << (odometry_inconsistent ? "REJECT" : "ACCEPT")
           << " reason: "
           << (odometry_inconsistent
                   ? "odometry inconsistent with global motion"
                   : "odometry consistent");
      LOG(INFO) << info.str();
      if (odometry_inconsistent) {
        return;
      }
    } else if (odometry_check.reverse_direction) {
      // Reverse direction: a frozen-map node matched against a live submap
      // (either in the local-search window or via full-submap matching). The
      // frozen trajectory has no odometry, so instead reject a match whose
      // correction is far larger than the trusted localization could plausibly
      // have drifted. Genuine relocalization is handled by the forward
      // direction (odometry-gated above).
      const transform::Rigid2d correction =
          initial_pose.inverse() * pose_estimate;
      const double correction_translation = correction.translation().norm();
      const double correction_rotation =
          std::abs(correction.normalized_angle());
      const bool correction_inconsistent =
          correction_translation >
              options_.global_localization_max_odometry_translation_error() ||
          correction_rotation >
              options_.global_localization_max_odometry_rotation_error();
      if (correction_inconsistent) {
        kGlobalConstraintsOdometryRejectedMetric->Increment();
      }
      std::ostringstream info;
      info << "[GlobalLoc] node: " << node_id << " submap: " << submap_id
           << " mode=" << (match_full_submap ? "GLOBAL" : "LOCAL-cross")
           << " reverse=true" << std::fixed
           << " scan_score: " << std::setprecision(4) << score
           << " correction_translation: " << std::setprecision(4)
           << correction_translation << " correction_rotation: "
           << std::setprecision(4) << correction_rotation
           << " decision: " << (correction_inconsistent ? "REJECT" : "ACCEPT")
           << " reason: "
           << (correction_inconsistent
                   ? "large correction inconsistent with trusted localization"
                   : "correction consistent");
      LOG(INFO) << info.str();
      if (correction_inconsistent) {
        return;
      }
    }
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  // Diagnostic: log every constraint that is actually created, local or global,
  // so a pose jump can be attributed to the correct match path. For global
  // matches this is preceded by the gate's ACCEPT/REJECT line above.
  {
    const transform::Rigid2d correction = initial_pose.inverse() * pose_estimate;
    LOG(INFO) << "[MatchResult] node " << node_id << " submap " << submap_id
              << " type=" << (match_full_submap ? "GLOBAL" : "LOCAL")
              << " score=" << score
              << " correction_translation=" << correction.translation().norm()
              << " correction_rotation=" << correction.normalized_angle();
  }
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    VLOG(1) << info.str();
  }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      VLOG(1) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      VLOG(1) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  kGlobalConstraintsOdometryRejectedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "odometry_rejected"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
