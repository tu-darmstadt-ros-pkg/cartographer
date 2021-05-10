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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_MULTI_RESOLUTION_TSDF_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_MULTI_RESOLUTION_TSDF_H_

#include <cmath>

#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

template <typename T, typename T2>
void InterpolateLinear(const HybridGridTSDF* tsdf, const T2& q1, const T2& q2,
                       const double w1, const double w2,
                       const T& normalized_ratio, T& q, double& w) {
  if (w1 == 0.0 && w2 == 0.0) {
    q = T(tsdf->ValueConverter().getMinTSD());
    w = 0.0;
  } else if (w1 == 0.0) {
    q = T(q2);
    w = w2;
  } else if (w2 == 0.0) {
    q = T(q1);
    w = w1;
  } else {
    q = (q2 - q1) * normalized_ratio + q1;
    w = w1 + w2;
  }
}

}  // namespace

// Interpolates between OccupancyGrid probability voxels. We use the tricubic
// interpolation which interpolates the values and has vanishing derivative at
// these points.
//
// This class is templated to work with the autodiff that Ceres provides.
// For this reason, it is also important that the interpolation scheme be
// continuously differentiable.
class InterpolatedMultiResolutionTSDF {
 public:
  explicit InterpolatedMultiResolutionTSDF(
      const std::vector<const HybridGridTSDF*>& tsdf_pyramid)
      : tsdf_pyramid_(tsdf_pyramid) {
    double last_resolution = 0.;
    for (const auto tsdf : tsdf_pyramid) {
      CHECK_LT(last_resolution, tsdf->resolution())
          << "Expecting TSDF pyramid to be sorted in ascending order.";
      last_resolution = tsdf->resolution();
    }
  }

  InterpolatedMultiResolutionTSDF(const InterpolatedMultiResolutionTSDF&) =
      delete;
  InterpolatedMultiResolutionTSDF& operator=(
      const InterpolatedMultiResolutionTSDF&) = delete;

  // Returns the interpolated probability at (x, y, z) of the OccupancyGrid
  // used to perform the interpolation.
  //
  // This is a piecewise, continuously differentiable function. We use the
  // scalar part of Jet parameters to select our interval below. It is the
  // tensor product volume of piecewise cubic polynomials that interpolate
  // the values, and have vanishing derivative at the interval boundaries.
  template <typename T>
  T GetTSD(const T& x, const T& y, const T& z) const {
    double x1, y1, z1, x2, y2, z2;
    for (const auto tsdf : tsdf_pyramid_) {
      ComputeInterpolationDataPoints(tsdf, x, y, z, &x1, &y1, &z1, &x2, &y2,
                                     &z2);
      const Eigen::Array3i index1 =
          tsdf->GetCellIndex(Eigen::Vector3f(x1, y1, z1));

      const double w111 = tsdf->GetWeight(index1);
      const double w112 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 0, 1));
      const double w121 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 1, 0));
      const double w122 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 1, 1));
      const double w211 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 0, 0));
      const double w212 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 0, 1));
      const double w221 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 1, 0));
      const double w222 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 1, 1));
      int max_num_invalid_octants = 0;
      int num_invalid_octants = int(w111 == 0.0) + int(w112 == 0.0) +
                                int(w121 == 0.0) + int(w122 == 0.0) +
                                int(w211 == 0.0) + int(w212 == 0.0) +
                                int(w221 == 0.0) + int(w222 == 0.0);
      if (num_invalid_octants > max_num_invalid_octants) {
        continue;
      }

      const double q111 = tsdf->GetTSD(index1);
      const double q112 = tsdf->GetTSD(index1 + Eigen::Array3i(0, 0, 1));
      const double q121 = tsdf->GetTSD(index1 + Eigen::Array3i(0, 1, 0));
      const double q122 = tsdf->GetTSD(index1 + Eigen::Array3i(0, 1, 1));
      const double q211 = tsdf->GetTSD(index1 + Eigen::Array3i(1, 0, 0));
      const double q212 = tsdf->GetTSD(index1 + Eigen::Array3i(1, 0, 1));
      const double q221 = tsdf->GetTSD(index1 + Eigen::Array3i(1, 1, 0));
      const double q222 = tsdf->GetTSD(index1 + Eigen::Array3i(1, 1, 1));

      const T normalized_x = (x - x1) / (x2 - x1);
      const T normalized_y = (y - y1) / (y2 - y1);
      const T normalized_z = (z - z1) / (z2 - z1);

      T q11, q12, q21, q22, q1, q2, q;
      double w11, w12, w21, w22, w1, w2, w;
      InterpolateLinear(tsdf, q111, q112, w111, w112, normalized_z, q11, w11);
      InterpolateLinear(tsdf, q121, q122, w121, w122, normalized_z, q12, w12);
      InterpolateLinear(tsdf, q211, q212, w211, w212, normalized_z, q21, w21);
      InterpolateLinear(tsdf, q221, q222, w221, w222, normalized_z, q22, w22);

      InterpolateLinear(tsdf, q11, q12, w11, w12, normalized_y, q1, w1);
      InterpolateLinear(tsdf, q21, q22, w21, w22, normalized_y, q2, w2);

      InterpolateLinear(tsdf, q1, q2, w1, w2, normalized_x, q, w);

      return q;
    }

    return T(tsdf_pyramid_.front()->ValueConverter().getMinTSD());
  }

  template <typename T>
  T GetWeight(const T& x, const T& y, const T& z) const {
    double x1, y1, z1, x2, y2, z2;
    for (const auto tsdf : tsdf_pyramid_) {
      ComputeInterpolationDataPoints(tsdf, x, y, z, &x1, &y1, &z1, &x2, &y2,
                                     &z2);
      const Eigen::Array3i index1 =
          tsdf->GetCellIndex(Eigen::Vector3f(x1, y1, z1));
      const double q111 = tsdf->GetWeight(index1);
      const double q112 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 0, 1));
      const double q121 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 1, 0));
      const double q122 = tsdf->GetWeight(index1 + Eigen::Array3i(0, 1, 1));
      const double q211 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 0, 0));
      const double q212 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 0, 1));
      const double q221 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 1, 0));
      const double q222 = tsdf->GetWeight(index1 + Eigen::Array3i(1, 1, 1));

      const T normalized_x = (x - x1) / (x2 - x1);
      const T normalized_y = (y - y1) / (y2 - y1);
      const T normalized_z = (z - z1) / (z2 - z1);

      const T q11 = (q112 - q111) * normalized_z + q111;
      const T q12 = (q122 - q121) * normalized_z + q121;
      const T q21 = (q212 - q211) * normalized_z + q211;
      const T q22 = (q222 - q221) * normalized_z + q221;

      const T q1 = (q12 - q11) * normalized_y + q11;
      const T q2 = (q22 - q21) * normalized_y + q21;
      LOG_EVERY_N(INFO, 1000) << "TODO(kdaun) weight similar to tsdf";
      return (q2 - q1) * normalized_x + q1;
    }
    return T(0.0);
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const HybridGridTSDF* tsdf, const T& x,
                                      const T& y, const T& z, double* x1,
                                      double* y1, double* z1, double* x2,
                                      double* y2, double* z2) const {
    const Eigen::Vector3f lower = CenterOfLowerVoxel(x, y, z, tsdf);
    *x1 = lower.x();
    *y1 = lower.y();
    *z1 = lower.z();
    *x2 = lower.x() + tsdf->resolution();
    *y2 = lower.y() + tsdf->resolution();
    *z2 = lower.z() + tsdf->resolution();
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y, z). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector3f CenterOfLowerVoxel(const double x, const double y,
                                     const double z,
                                     const HybridGridTSDF* tsdf) const {
    // Center of the cell containing (x, y, z).
    Eigen::Vector3f center =
        tsdf->GetCenterOfCell(tsdf->GetCellIndex(Eigen::Vector3f(x, y, z)));
    // Move to the next lower voxel center.
    if (center.x() > x) {
      center.x() -= tsdf->resolution();
    }
    if (center.y() > y) {
      center.y() -= tsdf->resolution();
    }
    if (center.z() > z) {
      center.z() -= tsdf->resolution();
    }
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector3f CenterOfLowerVoxel(const T& jet_x, const T& jet_y,
                                     const T& jet_z,
                                     const HybridGridTSDF* tsdf) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a, jet_z.a, tsdf);
  }

  const std::vector<const HybridGridTSDF*>& tsdf_pyramid_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_MULTI_RESOLUTION_TSDF_H_
