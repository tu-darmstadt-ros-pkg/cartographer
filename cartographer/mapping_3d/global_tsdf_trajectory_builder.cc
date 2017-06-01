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

#include "cartographer/mapping_3d/global_tsdf_trajectory_builder.h"

#include "cartographer/mapping_3d/local_tsdf_trajectory_builder.h"

namespace cartographer {
namespace mapping_3d {

GlobalTSDFTrajectoryBuilder::GlobalTSDFTrajectoryBuilder(
        const proto::LocalTrajectoryBuilderOptions& options,
        const int trajectory_id, SparsePoseGraphConversion* sparse_pose_graph)
    : trajectory_id_(trajectory_id),
      sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(CreateLocalTSDFTrajectoryBuilder(options)) {}

GlobalTSDFTrajectoryBuilder::~GlobalTSDFTrajectoryBuilder() {}

const mapping_3d::TSDFs* GlobalTSDFTrajectoryBuilder::submaps() const {
  return local_trajectory_builder_->submaps();
}

void GlobalTSDFTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_->AddImuData(time, linear_acceleration,
                                        angular_velocity);
  sparse_pose_graph_->AddImuData(trajectory_id_, time, linear_acceleration,
                                 angular_velocity);
}

void GlobalTSDFTrajectoryBuilder::AddRangefinderData(
    const common::Time time, const Eigen::Vector3f& origin,
    const sensor::PointCloud& ranges) {
  auto insertion_result =
      local_trajectory_builder_->AddRangefinderData(time, origin, ranges);

  if (insertion_result == nullptr) {
    return;
  }

  sparse_pose_graph_->AddScan(
      insertion_result->time, insertion_result->range_data_in_tracking,
      insertion_result->pose_observation, insertion_result->covariance_estimate,
      trajectory_id_, insertion_result->matching_submap,
      insertion_result->insertion_submaps);
}

void GlobalTSDFTrajectoryBuilder::AddOdometerData(const common::Time time,
                                              const transform::Rigid3d& pose) {
  local_trajectory_builder_->AddOdometerData(time, pose);
}

const GlobalTSDFTrajectoryBuilder::PoseEstimate&
GlobalTSDFTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_->pose_estimate();
}

}  // namespace mapping_3d
}  // namespace cartographer
