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

#ifndef CARTOGRAPHER_MAPPING_3D_ROBUST_OPTIMIZING_VOXBLOX_ESDF_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_ROBUST_OPTIMIZING_VOXBLOX_ESDF_LOCAL_TRAJECTORY_BUILDER_H_

#include <array>
#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/imu_integration.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping_3d/local_voxblox_esdf_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {

// Batches up some sensor data and optimizes them in one go to get a locally
// consistent trajectory.
class RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder
    : public LocalVoxbloxESDFTrajectoryBuilderInterface {
 public:
  explicit RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder() override;

  RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder(const RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder&) =
      delete;
  RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder& operator=(
      const RobustOptimizingVoxbloxESDFLocalTrajectoryBuilder&) = delete;

  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;
  std::unique_ptr<InsertionResult> AddRangefinderData(
      common::Time time, const Eigen::Vector3f& origin,
      const sensor::PointCloud& ranges) override;
  void AddOdometerData(common::Time time,
                       const transform::Rigid3d& pose) override;
  const mapping_3d::VoxbloxESDFs* submaps() const override;
  const PoseEstimate& pose_estimate() const override;

 private:
  struct State {
    std::array<double, 3> translation;
    std::array<double, 4> rotation;  // Rotation quaternion as (w, x, y, z).
    std::array<double, 3> velocity;

    State(const Eigen::Vector3d& translation,
          const Eigen::Quaterniond& rotation, const Eigen::Vector3d& velocity)
        : translation{{translation.x(), translation.y(), translation.z()}},
          rotation{{rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
          velocity{{velocity.x(), velocity.y(), velocity.z()}} {}

    Eigen::Quaterniond ToQuaternion() const {
      return Eigen::Quaterniond(rotation[0], rotation[1], rotation[2],
                                rotation[3]);
    }

    transform::Rigid3d ToRigid() const {
      return transform::Rigid3d(
          Eigen::Vector3d(translation[0], translation[1], translation[2]),
          ToQuaternion());
    }
  };

  struct Batch {
    common::Time time;
    sensor::PointCloud points;
    sensor::PointCloud high_resolution_filtered_points;
    sensor::PointCloud low_resolution_filtered_points;
    State state;
    double delay_imu;
  };

  struct OdometerData {
    common::Time time;

    // Dead-reckoning pose of the odometry.
    transform::Rigid3d pose;
  };

  State PredictState(const State& start_state, const common::Time start_time,
                     const common::Time end_time);

  void RemoveObsoleteSensorData();

  std::unique_ptr<InsertionResult> AddAccumulatedRangeData(
      common::Time time, const transform::Rigid3d& pose_observation,
      const sensor::RangeData& range_data_in_tracking, const Eigen::Vector3f& sensor_origin, const transform::Rigid3f& world_to_sensor);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(const common::Time time, const sensor::RangeData& range_data_in_tracking,
      const transform::Rigid3d& pose_observation, const Eigen::Vector3f& sensor_origin, const transform::Rigid3f &world_to_sensor);

  void TransformStates(const transform::Rigid3d& transform);
  std::unique_ptr<InsertionResult> MaybeOptimize(common::Time time, const Eigen::Vector3f& sensor_origin);

  const proto::LocalTrajectoryBuilderOptions options_;
  const ceres::Solver::Options ceres_solver_options_;
  std::unique_ptr<mapping_3d::VoxbloxESDFs> submaps_;
  int num_accumulated_;
  int num_update_scans_;

  std::deque<Batch> batches_;
  double gravity_constant_ = 9.8;
  std::deque<ImuData> imu_data_;
  std::deque<OdometerData> odometer_data_;
  std::unique_ptr<mapping::ImuTracker> imu_tracker_;

  PoseEstimate last_pose_estimate_;

  MotionFilter motion_filter_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_ROBUST_OPTIMIZING_VOXBLOX_ESDF_LOCAL_TRAJECTORY_BUILDER_H_
