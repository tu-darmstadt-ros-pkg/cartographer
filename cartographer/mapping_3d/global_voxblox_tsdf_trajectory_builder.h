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

#ifndef CARTOGRAPHER_MAPPING_3D_GLOBAL_VOXBLOX_TSDF_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_GLOBAL_VOXBLOX_TSDF_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/kalman_tsdf_local_trajectory_builder.h"
#include "cartographer/mapping_3d/optimizing_tsdf_local_trajectory_builder.h"

#include "cartographer/mapping_3d/local_voxblox_tsdf_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/sparse_pose_graph_voxblox_conversion.h"

namespace cartographer {
namespace mapping_3d {

class GlobalVoxbloxTSDFTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface {
 public:
  GlobalVoxbloxTSDFTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options,
                              int trajectory_id,
                              mapping_3d::SparsePoseGraphVoxbloxConversion* sparse_pose_graph);
  ~GlobalVoxbloxTSDFTrajectoryBuilder() override;

  GlobalVoxbloxTSDFTrajectoryBuilder(const GlobalVoxbloxTSDFTrajectoryBuilder&) = delete;
  GlobalVoxbloxTSDFTrajectoryBuilder& operator=(const GlobalVoxbloxTSDFTrajectoryBuilder&) = delete;

  const mapping_3d::VoxbloxTSDFs* submaps() const override;
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;
  void AddRangefinderData(common::Time time, const Eigen::Vector3f& origin,
                          const sensor::PointCloud& ranges) override;
  void AddOdometerData(common::Time time,
                       const transform::Rigid3d& pose) override;
  const PoseEstimate& pose_estimate() const override;

 private:
  const int trajectory_id_;
  mapping_3d::SparsePoseGraphVoxbloxConversion* const sparse_pose_graph_;
  std::unique_ptr<LocalVoxbloxTSDFTrajectoryBuilderInterface> local_trajectory_builder_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_GLOBAL_VOXBLOX_TSDF_TRAJECTORY_BUILDER_H_
