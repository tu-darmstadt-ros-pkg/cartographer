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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TSDF_2D_CONSTRAINED_OCCUPIED_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TSDF_2D_CONSTRAINED_OCCUPIED_SPACE_COST_FUNCTOR_H_

#include <open_chisel/Chisel.h>

#include "Eigen/Core"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/scan_matching/interpolated_tsdf.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class TSDF2DConstrainedOccupiedSpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  TSDF2DConstrainedOccupiedSpaceCostFunctor(const double scaling_factor,
                           const sensor::PointCloud& point_cloud,
                           chisel::ChiselConstPtr<chisel::DistVoxel> tsdf,
                           Eigen::Matrix<float, 2, 3> projection,
                           Eigen::Vector3f rotation_axis,
                           int coarsening_factor, float max_truncation_distance)
      : scaling_factor_(scaling_factor),
        coarsening_factor_(coarsening_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(tsdf, max_truncation_distance),
        projection_(projection),
        rotation_axis_(rotation_axis) {}

  TSDF2DConstrainedOccupiedSpaceCostFunctor(const TSDF2DConstrainedOccupiedSpaceCostFunctor&) = delete;
  TSDF2DConstrainedOccupiedSpaceCostFunctor& operator=(const TSDF2DConstrainedOccupiedSpaceCostFunctorj&) = delete;

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation_2d(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation_2d(pose[2]);
    Eigen::Matrix<T, 3, 1> translation_3d = translation_2d*projection_;
    //todo(kdaun) check transforms (jet correct?)

    const transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation_3d),
        Eigen::Quaternion<T>(rotation_axis_.x() * sin(pose[2]/2.), rotation_axis_.y() * sin(pose[2]/2.),
            rotation_axis_.z() * sin(pose[2]/2.), cos(pose[2]/2.)));
    return Evaluate(transform, residual);
  }

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> world =
          transform * point_cloud_[i].cast<T>();

      const T sdf = interpolated_grid_.GetSDF(world[0], world[1], world[2], coarsening_factor_);
      residual[i] = scaling_factor_ * sdf;
    }
    return true;
  }

 private:
  const double scaling_factor_;
  const int coarsening_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedTSDF interpolated_grid_;  
  const Eigen::Matrix<float, 2, 3> projection_;
  const Eigen::Vector3f rotation_axis_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TSDF_2D_CONSTRAINED_OCCUPIED_SPACE_COST_FUNCTOR_H_
