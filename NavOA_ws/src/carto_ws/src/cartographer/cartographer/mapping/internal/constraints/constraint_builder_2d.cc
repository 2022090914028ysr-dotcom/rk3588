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

  const bool cross_trajectory =
      node_id.trajectory_id != submap_id.trajectory_id;
  const bool is_loop_closure = match_full_submap || cross_trajectory;

  // Stage 2 (kidnap-aware gating): while the robot is airborne, do not create
  // loop-closure (global or cross-trajectory) constraints at all. Reset any
  // in-progress recovery confirmation so a fresh episode starts once the robot
  // is placed back on the ground.
  if (is_loop_closure && odometry_check.kidnap_active &&
      !odometry_check.robot_on_ground) {
    ResetRecoveryConfirmation();
    ResetRelocalizationState();
    LOG(INFO) << "[GLOBAL][KIDNAP-LIFTED] node: " << node_id
              << " submap: " << submap_id
              << " decision=SKIP reason=robot_not_on_ground";
    return;
  }

  // RECOVERY: the robot has been lifted and placed down again, and the
  // localization has not yet been re-confirmed. In this mode the odometry
  // consistency gate (Stage 1) is replaced by a stronger scan-based validation.
  const bool recovery_unconfirmed =
      is_loop_closure && odometry_check.kidnap_active &&
      odometry_check.robot_on_ground && !RecoveryConfirmed();

  // INITIAL LOCALIZATION: no loop closure has yet tied these two trajectories
  // together, so the odometry gate has no trustworthy prior to compare against.
  // The single-best full-submap match is unguarded here and, in a repetitive
  // environment, can land on the wrong similar structure (which often scores
  // HIGHER than the correct one).
  const bool initial_localization =
      is_loop_closure && !odometry_check.localization_established;

  // Without a trustworthy prior (startup or kidnap recovery) we require the
  // stronger scan-based ambiguity validation instead of the odometry gate.
  const bool no_trustworthy_prior = recovery_unconfirmed || initial_localization;

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (no_trustworthy_prior) {
      // Startup or kidnap recovery: there is no trustworthy pose-graph prior,
      // so a single per-submap match cannot be trusted in a symmetric
      // environment. Record this submap's best match and defer the accept /
      // reject decision to ResolvePendingGlobalMatches(), where the best match
      // across ALL submaps for this node is available and cross-submap
      // ambiguity can be assessed.
      if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
              constant_data->filtered_gravity_aligned_point_cloud,
              options_.global_localization_min_score(), &score,
              &pose_estimate)) {
        absl::MutexLock locker(&mutex_);
        pending_global_matches_.push_back(
            {node_id, submap_id, score, pose_estimate, submap, constant_data,
             recovery_unconfirmed});
      }
      return;
    } else if (submap_scan_matcher.fast_correlative_scan_matcher
                   ->MatchFullSubmap(
                       constant_data->filtered_gravity_aligned_point_cloud,
                       options_.global_localization_min_score(), &score,
                       &pose_estimate)) {
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
  // The gate only fires once the localization has actually been established
  // (the trajectories tied by a real loop closure). Before that the pose-graph
  // estimate is the unlocalized initial guess, so any correct relocalization
  // implies a large correction and must be allowed through. It is also skipped
  // while a kidnap recovery is still unconfirmed, since the scan-based recovery
  // validation above has already decided in that case.
  if (!recovery_unconfirmed && odometry_check.localization_established &&
      is_loop_closure &&
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
        // A full-submap global match that is very confident despite disagreeing
        // with odometry is likely a genuine re-detection of a distinctive place
        // after the localization drifted to the wrong similar structure. Route
        // it through temporal confirmation instead of rejecting it outright, so
        // the robot can self-correct once it moves somewhere distinctive.
        if (match_full_submap &&
            score >= options_.global_localization_relocalization_min_score()) {
          {
            absl::MutexLock locker(&relocalization_mutex_);
            relocalization_state_.active = true;
          }
          absl::MutexLock locker(&mutex_);
          pending_global_matches_.push_back(
              {node_id, submap_id, score, pose_estimate, submap, constant_data,
               false, /* recovery */
               true /* relocalization */});
          LOG(INFO) << "[GlobalLoc] node: " << node_id
                    << " submap: " << submap_id
                    << " decision=DEFER reason=relocalization_candidate";
        }
        return;
      }
      // Odometry-consistent full-submap global match: the robot is tracking fine
      // at its current place. Record this so a relocalization candidate for the
      // same node is recognized as a symmetric-room fluke and suppressed.
      if (match_full_submap) {
        absl::MutexLock locker(&mutex_);
        nodes_with_consistent_global_match_.insert(node_id);
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

bool ConstraintBuilder2D::RecoveryConfirmed() {
  absl::MutexLock locker(&recovery_mutex_);
  return recovery_state_.confirmed;
}

bool ConstraintBuilder2D::RecoveryUnconfirmed() {
  absl::MutexLock locker(&recovery_mutex_);
  return !recovery_state_.confirmed;
}

void ConstraintBuilder2D::ResetRecoveryConfirmation() {
  absl::MutexLock locker(&recovery_mutex_);
  recovery_state_.confirmed = false;
  recovery_state_.hypothesis_set = false;
  recovery_state_.hypothesis = transform::Rigid2d::Identity();
  recovery_state_.consecutive_count = 0;
}

bool ConstraintBuilder2D::RelocalizationInProgress() {
  absl::MutexLock locker(&relocalization_mutex_);
  return relocalization_state_.active;
}

void ConstraintBuilder2D::ResetRelocalizationState() {
  absl::MutexLock locker(&relocalization_mutex_);
  relocalization_state_.active = false;
  relocalization_state_.hypothesis_set = false;
  relocalization_state_.hypothesis = transform::Rigid2d::Identity();
  relocalization_state_.consecutive_count = 0;
}

void ConstraintBuilder2D::ResolvePendingGlobalMatches(Result* result) {
  std::vector<PendingGlobalMatch> pending;
  std::set<NodeId> consistent_nodes;
  {
    absl::MutexLock locker(&mutex_);
    pending.swap(pending_global_matches_);
    consistent_nodes.swap(nodes_with_consistent_global_match_);
  }
  // If no relocalization candidate arrived this cycle, any in-flight attempt has
  // either succeeded (the pose snapped and matches are odometry-consistent
  // again) or stalled (the robot left the distinctive area). Stop forcing
  // full-submap global matches.
  bool has_relocalization = false;
  for (const PendingGlobalMatch& m : pending) {
    if (m.relocalization) {
      has_relocalization = true;
      break;
    }
  }
  if (!has_relocalization) {
    absl::MutexLock locker(&relocalization_mutex_);
    relocalization_state_.active = false;
  }
  if (pending.empty()) {
    return;
  }

  // Sort by (node_id, score descending) so each node's matches are contiguous
  // and its best match comes first.
  std::sort(pending.begin(), pending.end(),
            [](const PendingGlobalMatch& a, const PendingGlobalMatch& b) {
              if (a.node_id != b.node_id) {
                return a.node_id < b.node_id;
              }
              return a.score > b.score;
            });

  for (size_t begin = 0; begin < pending.size();) {
    size_t end = begin;
    while (end < pending.size() &&
           pending[end].node_id == pending[begin].node_id) {
      ++end;
    }

    const NodeId node_id = pending[begin].node_id;
    const PendingGlobalMatch& top1 = pending[begin];
    const bool recovery = top1.recovery;
    const bool relocalization = top1.relocalization;
    const char* tag = recovery ? "[GLOBAL][KIDNAP-RECOVERY]"
                     : relocalization ? "[GLOBAL][RELOCALIZATION]"
                                      : "[GLOBAL][STARTUP]";

    // Cross-submap ambiguity: only a runner-up that is BOTH spatially separated
    // from the best candidate (a genuinely different hypothesis, e.g. a
    // symmetric twin room) AND scores within the margin counts as ambiguous.
    // Submaps that overlap the same physical location naturally score close
    // together and must NOT be treated as ambiguity — they are the same
    // hypothesis seen through adjacent submaps (common near the map origin).
    const PendingGlobalMatch* separated_runner_up = nullptr;
    for (size_t i = begin + 1; i < end; ++i) {
      const double separation =
          (pending[i].pose_estimate.translation() -
           top1.pose_estimate.translation())
              .norm();
      const double angular_separation = std::abs(common::NormalizeAngleDifference(
          pending[i].pose_estimate.rotation().angle() -
          top1.pose_estimate.rotation().angle()));
      if (separation >
              options_.global_localization_recovery_min_separation() ||
          angular_separation >
              options_.global_localization_recovery_min_angular_separation()) {
        separated_runner_up = &pending[i];
        break;
      }
    }
    const bool has_runner_up = separated_runner_up != nullptr;
    const float top2_score =
        has_runner_up ? separated_runner_up->score
                      : -std::numeric_limits<float>::infinity();
    const float score_margin = top1.score - top2_score;

    if (has_runner_up &&
        score_margin <
            options_.global_localization_recovery_min_score_margin()) {
      if (recovery) {
        ResetRecoveryConfirmation();
      }
      if (relocalization) {
        ResetRelocalizationState();
      }
      LOG(INFO) << tag << " node: " << node_id
                << " top1_submap=" << top1.submap_id
                << " top1_score=" << top1.score
                << " top2_submap=" << separated_runner_up->submap_id
                << " top2_score=" << top2_score
                << " score_margin=" << score_margin
                << " decision=REJECT reason=AMBIGUOUS";
      begin = end;
      continue;
    }

    if (recovery) {
      // Temporal confirmation across consecutive nodes: the accepted hypothesis
      // must stay put within confirmation_distance for num_confirmations nodes.
      bool confirmed = false;
      std::string reject_reason;
      {
        absl::MutexLock locker(&recovery_mutex_);
        if (recovery_state_.hypothesis_set) {
          const double hypothesis_distance =
              (top1.pose_estimate.translation() -
               recovery_state_.hypothesis.translation())
                  .norm();
          if (hypothesis_distance >
              options_.global_localization_recovery_confirmation_distance()) {
            recovery_state_.hypothesis = top1.pose_estimate;
            recovery_state_.consecutive_count = 1;
            reject_reason = "UNSTABLE";
          } else {
            recovery_state_.hypothesis = top1.pose_estimate;
            ++recovery_state_.consecutive_count;
          }
        } else {
          recovery_state_.hypothesis = top1.pose_estimate;
          recovery_state_.hypothesis_set = true;
          recovery_state_.consecutive_count = 1;
        }
        if (recovery_state_.consecutive_count >=
            options_.global_localization_recovery_num_confirmations()) {
          recovery_state_.confirmed = true;
          confirmed = true;
        }
      }
      if (!confirmed) {
        if (reject_reason.empty()) {
          reject_reason = "PENDING_CONFIRMATION";
        }
        LOG(INFO) << tag << " node: " << node_id
                  << " top1_submap=" << top1.submap_id
                  << " top1_score=" << top1.score
                  << " top2_score=" << top2_score
                  << " score_margin=" << score_margin
                  << " decision=REJECT reason=" << reject_reason;
        begin = end;
        continue;
      }
    } else if (relocalization) {
      // The robot produced an odometry-consistent full-submap global match for
      // this node, i.e. it is tracking fine at its current place. The
      // contradictory candidate is a symmetric-room fluke, not a genuine
      // wrong-lock, so suppress it.
      if (consistent_nodes.count(node_id) > 0) {
        ResetRelocalizationState();
        LOG(INFO) << tag << " node: " << node_id
                  << " top1_submap=" << top1.submap_id
                  << " top1_score=" << top1.score
                  << " top2_score=" << top2_score
                  << " score_margin=" << score_margin
                  << " decision=REJECT reason=TRACKING_CONSISTENT";
        begin = end;
        continue;
      }
      // Temporal confirmation of a contradictory-but-confident global match:
      // consecutive nodes must point to the same (spatially consistent) target
      // before the relocalization jump is accepted, so a single symmetric-room
      // fluke cannot pull the pose graph to a wrong place.
      bool confirmed = false;
      std::string reject_reason;
      {
        absl::MutexLock locker(&relocalization_mutex_);
        if (relocalization_state_.hypothesis_set) {
          const double hypothesis_distance =
              (top1.pose_estimate.translation() -
               relocalization_state_.hypothesis.translation())
                  .norm();
          if (hypothesis_distance >
              options_.global_localization_recovery_confirmation_distance()) {
            relocalization_state_.hypothesis = top1.pose_estimate;
            relocalization_state_.consecutive_count = 1;
            reject_reason = "UNSTABLE";
          } else {
            relocalization_state_.hypothesis = top1.pose_estimate;
            ++relocalization_state_.consecutive_count;
          }
        } else {
          relocalization_state_.hypothesis = top1.pose_estimate;
          relocalization_state_.hypothesis_set = true;
          relocalization_state_.consecutive_count = 1;
        }
        if (relocalization_state_.consecutive_count >=
            options_.global_localization_recovery_num_confirmations()) {
          confirmed = true;
          // Reset so a future, genuinely different relocalization starts fresh.
          relocalization_state_.hypothesis_set = false;
          relocalization_state_.consecutive_count = 0;
          relocalization_state_.active = false;
        }
      }
      if (!confirmed) {
        if (reject_reason.empty()) {
          reject_reason = "PENDING_CONFIRMATION";
        }
        LOG(INFO) << tag << " node: " << node_id
                  << " top1_submap=" << top1.submap_id
                  << " top1_score=" << top1.score
                  << " top2_score=" << top2_score
                  << " score_margin=" << score_margin
                  << " decision=REJECT reason=" << reject_reason;
        begin = end;
        continue;
      }
    }

    // Unambiguous winner: refine with Ceres and emit the constraint.
    PendingGlobalMatch accepted = top1;
    ceres::Solver::Summary unused_summary;
    ceres_scan_matcher_.Match(
        accepted.pose_estimate.translation(), accepted.pose_estimate,
        accepted.constant_data->filtered_gravity_aligned_point_cloud,
        *accepted.submap->grid(), &accepted.pose_estimate, &unused_summary);
    const transform::Rigid2d constraint_transform =
        ComputeSubmapPose(*accepted.submap).inverse() * accepted.pose_estimate;
    kGlobalConstraintsFoundMetric->Increment();
    kGlobalConstraintScoresMetric->Observe(accepted.score);

    LOG(INFO) << tag << " node: " << node_id
              << " top1_submap=" << top1.submap_id
              << " top1_score=" << top1.score
              << " top2_score=" << top2_score
              << " score_margin=" << score_margin
              << " decision=ACCEPT reason="
              << (recovery ? "RECOVERY_CONFIRMED"
                           : relocalization ? "RELOCALIZATION_CONFIRMED"
                                            : "UNAMBIGUOUS");

    result->push_back(Constraint{
        accepted.submap_id, accepted.node_id,
        {transform::Embed3D(constraint_transform),
         options_.loop_closure_translation_weight(),
         options_.loop_closure_rotation_weight()},
        Constraint::INTER_SUBMAP});

    begin = end;
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
  ResolvePendingGlobalMatches(&result);
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
