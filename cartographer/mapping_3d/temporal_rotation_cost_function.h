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

#ifndef CARTOGRAPHER_MAPPING_3D_TEMPORAL_ROTATION_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_TEMPORAL_ROTATION_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_3d/imu_integration.h"

namespace cartographer {
namespace mapping_3d {

// Penalizes differences between IMU data and optimized orientations.
class TemporalRotationCostFunction {
 public:
  TemporalRotationCostFunction(const double rotation_scaling_factor,
                       const double time_scaling_factor,
                       const std::deque<ImuData>& imu_data,
                       const common::Time& time_start,
                       const common::Time& time_end)
      : rotation_scaling_factor_(rotation_scaling_factor),
        time_scaling_factor_(time_scaling_factor),
        imu_data_(imu_data),
        time_start_(time_start),
        time_end_(time_end) {}

  TemporalRotationCostFunction(const TemporalRotationCostFunction&) = delete;
  TemporalRotationCostFunction& operator=(const TemporalRotationCostFunction&) = delete;

  template <typename T>
  bool operator()(const T* const start_rotation, const T* const end_rotation,
                  const T* const start_delay, const T* const end_delay,
                  T* residual) const {
    auto it = imu_data_.cbegin();

    const IntegrateImuResult<T> result = IntegrateImuExperimental<T>(imu_data_,
         Eigen::Transform<T, 3, Eigen::Affine>::Identity(),
         Eigen::Transform<T, 3, Eigen::Affine>::Identity(),
         time_start_,start_delay[0],
         (T)(common::ToSeconds(time_end_-time_start_)) + end_delay[0],
         &it);

    const Eigen::Quaternion<T> delta_rotation_imu_frame_ = result.delta_rotation;

    const Eigen::Quaternion<T> start(start_rotation[0], start_rotation[1],
                                     start_rotation[2], start_rotation[3]);
    const Eigen::Quaternion<T> end(end_rotation[0], end_rotation[1],
                                   end_rotation[2], end_rotation[3]);
    const Eigen::Quaternion<T> error =
        end.conjugate() * start * delta_rotation_imu_frame_;
    residual[0] = rotation_scaling_factor_ * error.x();
    residual[1] = rotation_scaling_factor_ * error.y();
    residual[2] = rotation_scaling_factor_ * error.z();
    residual[3] = time_scaling_factor_ * (end_delay[0] - start_delay[0]);
    return true;
  }

/*
    bool operator()(const double* const start_rotation, const double* const end_rotation,
                    const double* const start_delay, const double* const end_delay,
                    double* residual) const {

      auto it = imu_data_.cbegin();
      common::Time corrected_time_start = time_start_ + common::FromSeconds(start_delay[0]);
      common::Time corrected_time_end = time_end_ + common::FromSeconds(end_delay[0]);
      if (corrected_time_end - corrected_time_start < common::FromSeconds(0.0)) {
         //LOG(INFO)<<"time implausibility ";
        return false;
      }
      const IntegrateImuResult<double> result =
          IntegrateImu(imu_data_, corrected_time_start, corrected_time_end, &it);

      const Eigen::Quaterniond delta_rotation_imu_frame_ = result.delta_rotation;

      const Eigen::Quaterniond start(start_rotation[0], start_rotation[1],
                                       start_rotation[2], start_rotation[3]);
      const Eigen::Quaterniond end(end_rotation[0], end_rotation[1],
                                     end_rotation[2], end_rotation[3]);
      const Eigen::Quaterniond error =
          end.conjugate() * start * delta_rotation_imu_frame_;
      residual[0] = scaling_factor_ * error.x();
      residual[1] = scaling_factor_ * error.y();
      residual[2] = scaling_factor_ * error.z();
      residual[3] = 1e5 * (end_delay[0] - start_delay[0]); //todo(kdaun) separate scaling factor
      return true;
    }*/

 private:
  const double rotation_scaling_factor_;
  const double time_scaling_factor_;
  std::deque<ImuData> imu_data_;
  common::Time time_start_;
  common::Time time_end_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_TEMPORAL_ROTATION_COST_FUNCTION_H_

