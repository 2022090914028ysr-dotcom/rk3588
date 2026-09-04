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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/task.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace constraints {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap);

// Immutable inputs for the odometry-consistency gate applied to global
// localization. Computed on the pose graph thread (which owns the odometry
// data) and passed by value into the asynchronous constraint computation so
// that no mutable state is shared across threads.
struct GlobalLocalizationOdometry {
  // Whether a valid previous node and odometry interval are available. When
  // false the odometry gate is a no-op and the candidate is accepted.
  bool has_odometry = false;

  // Whether the node's and submap's trajectories have already been tied
  // together by a real loop-closure constraint (i.e. the localization is
  // established and the current pose-graph estimate is trustworthy). Until this
  // is true the gate is disabled entirely: at startup the estimate is still the
  // unlocalized initial guess, so a correct relocalization implies a large
  // correction and must not be rejected.
  bool localization_established = false;

  // Whether this match is in the reverse direction: a frozen-map node (no
  // odometry) matched against a live localization submap (with odometry). Such
  // matches cannot be odometry-gated, so the gate falls back to a
  // correction-magnitude check that assumes the trusted localization is already
  // roughly correct.
  bool reverse_direction = false;

  // Map-frame (gravity-aligned) global pose of the node immediately preceding
  // the node being localized.
  transform::Rigid2d previous_global_pose_2d = transform::Rigid2d::Identity();

  // Odometry motion from the previous node to the current node, expressed in
  // the previous node's gravity-aligned frame (directly comparable to
  // previous_global_pose_2d.inverse() * candidate_pose).
  transform::Rigid2d relative_odometry_2d = transform::Rigid2d::Identity();
};

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to 'MaybeAddConstraint',
// 'MaybeAddGlobalConstraint', and 'NotifyEndOfNode', then call 'WhenDone' once.
// After all computations are done the 'callback' will be called with the result
// and another MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder2D {
 public:
  using Constraint = PoseGraphInterface::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder2D(const proto::ConstraintBuilderOptions& options,
                      common::ThreadPoolInterface* thread_pool);
  ~ConstraintBuilder2D();

  ConstraintBuilder2D(const ConstraintBuilder2D&) = delete;
  ConstraintBuilder2D& operator=(const ConstraintBuilder2D&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // 'odometry_check' provides the (immutable) odometry-consistency inputs used
  // to reject a suspicious cross-trajectory loop closure before a constraint is
  // created. It is only populated for cross-trajectory matches and is a no-op
  // (empty) for same-trajectory local refinements.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose,
                          const GlobalLocalizationOdometry& odometry_check);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // 'initial_relative_pose' is the current (submap <- node) estimate from the
  // pose graph. It is not used to seed the full-submap search, but provides the
  // reference against which the matched pose is compared by the
  // reverse-direction correction-magnitude gate.
  //
  // 'odometry_check' provides the (immutable) odometry-consistency inputs used
  // to reject a suspicious global localization candidate before a constraint is
  // created. It is copied into the asynchronous task.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap2D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data,
      const transform::Rigid2d& initial_relative_pose,
      const GlobalLocalizationOdometry& odometry_check);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by 'MaybeAdd*Constraint' have finished.
  // 'callback' is executed in the 'ThreadPool'.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  struct SubmapScanMatcher {
    const Grid2D* grid = nullptr;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;
    std::weak_ptr<common::Task> creation_task_handle;
  };

  // The returned 'grid' and 'fast_correlative_scan_matcher' must only be
  // accessed after 'creation_task_handle' has completed.
  const SubmapScanMatcher* DispatchScanMatcherConstruction(
      const SubmapId& submap_id, const Grid2D* grid)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(const SubmapId& submap_id, const Submap2D* submap,
                         const NodeId& node_id, bool match_full_submap,
                         const TrajectoryNode::Data* const constant_data,
                         const transform::Rigid2d& initial_relative_pose,
                         const SubmapScanMatcher& submap_scan_matcher,
                         const GlobalLocalizationOdometry& odometry_check,
                         std::unique_ptr<Constraint>* constraint)
      ABSL_LOCKS_EXCLUDED(mutex_);

  void RunWhenDoneCallback() ABSL_LOCKS_EXCLUDED(mutex_);

  const constraints::proto::ConstraintBuilderOptions options_;
  common::ThreadPoolInterface* thread_pool_;
  absl::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      ABSL_GUARDED_BY(mutex_);

  // TODO(gaschler): Use atomics instead of mutex to access these counters.
  // Number of the node in reaction to which computations are currently
  // added. This is always the number of nodes seen so far, even when older
  // nodes are matched against a new submap.
  int num_started_nodes_ ABSL_GUARDED_BY(mutex_) = 0;

  int num_finished_nodes_ ABSL_GUARDED_BY(mutex_) = 0;

  std::unique_ptr<common::Task> finish_node_task_ ABSL_GUARDED_BY(mutex_);

  std::unique_ptr<common::Task> when_done_task_ ABSL_GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries. Constraint search results
  // with below-threshold scores are also 'nullptr'.
  std::deque<std::unique_ptr<Constraint>> constraints_ ABSL_GUARDED_BY(mutex_);

  // Map of dispatched or constructed scan matchers by 'submap_id'.
  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
      ABSL_GUARDED_BY(mutex_);
  std::map<SubmapId, common::FixedRatioSampler> per_submap_sampler_;

  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  common::Histogram score_histogram_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
