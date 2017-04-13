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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_TSDF_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_TSDF_H_

#include <cmath>

#include <open_chisel/Chisel.h>

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Interpolates between HybridGrid probability voxels. We use the tricubic
// interpolation which interpolates the values and has vanishing derivative at
// these points.
//
// This class is templated to work with the autodiff that Ceres provides.
// For this reason, it is also important that the interpolation scheme be
// continuously differentiable.



class InterpolatedTSDF {
 public:
  explicit InterpolatedTSDF(const chisel::ChiselPtr tsdf)
      : tsdf_(tsdf) {}

  InterpolatedTSDF(const InterpolatedTSDF&) = delete;
  InterpolatedTSDF& operator=(const InterpolatedTSDF&) = delete;

  // Returns the interpolated probability at (x, y, z) of the HybridGrid
  // used to perform the interpolation.
  //
  // This is a piecewise, continuously differentiable function. We use the
  // scalar part of Jet parameters to select our interval below. It is the
  // tensor product volume of piecewise cubic polynomials that interpolate
  // the values, and have vanishing derivative at the interval boundaries.


  template <typename T>
  T GetSDF(const T& x, const T& y, const T& z, int coarsening_factor) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2, coarsening_factor);

    const chisel::ChunkManager& chunk_manager = tsdf_->GetChunkManager();
    const chisel::DistVoxel* v111 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x1,y1,z1), coarsening_factor);
    const chisel::DistVoxel* v112 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x1,y1,z2), coarsening_factor);
    const chisel::DistVoxel* v121 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x1,y2,z1), coarsening_factor);
    const chisel::DistVoxel* v122 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x1,y2,z2), coarsening_factor);
    const chisel::DistVoxel* v211 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x2,y1,z1), coarsening_factor);
    const chisel::DistVoxel* v212 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x2,y1,z2), coarsening_factor);
    const chisel::DistVoxel* v221 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x2,y2,z1), coarsening_factor);
    const chisel::DistVoxel* v222 = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x2,y2,z2), coarsening_factor);

    const double q111 = getVoxelSDF(v111);
    const double q112 = getVoxelSDF(v112);
    const double q121 = getVoxelSDF(v121);
    const double q122 = getVoxelSDF(v122);
    const double q211 = getVoxelSDF(v211);
    const double q212 = getVoxelSDF(v212);
    const double q221 = getVoxelSDF(v221);
    const double q222 = getVoxelSDF(v222);

    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);
    const T normalized_z = (z - z1) / (z2 - z1);

    // Compute pow(..., 2) and pow(..., 3). Using pow() here is very expensive.
    const T normalized_xx = normalized_x * normalized_x;
    const T normalized_xxx = normalized_x * normalized_xx;
    const T normalized_yy = normalized_y * normalized_y;
    const T normalized_yyy = normalized_y * normalized_yy;
    const T normalized_zz = normalized_z * normalized_z;
    const T normalized_zzz = normalized_z * normalized_zz;

    // We first interpolate in z, then y, then x. All 7 times this uses the same
    // scheme: A * (2t^3 - 3t^2 + 1) + B * (-2t^3 + 3t^2).
    // The first polynomial is 1 at t=0, 0 at t=1, the second polynomial is 0
    // at t=0, 1 at t=1. Both polynomials have derivative zero at t=0 and t=1.
    const T q11 = (q111 - q112) * normalized_zzz * 2. +
                  (q112 - q111) * normalized_zz * 3. + q111;
    const T q12 = (q121 - q122) * normalized_zzz * 2. +
                  (q122 - q121) * normalized_zz * 3. + q121;
    const T q21 = (q211 - q212) * normalized_zzz * 2. +
                  (q212 - q211) * normalized_zz * 3. + q211;
    const T q22 = (q221 - q222) * normalized_zzz * 2. +
                  (q222 - q221) * normalized_zz * 3. + q221;
    const T q1 = (q11 - q12) * normalized_yyy * 2. +
                 (q12 - q11) * normalized_yy * 3. + q11;
    const T q2 = (q21 - q22) * normalized_yyy * 2. +
                 (q22 - q21) * normalized_yy * 3. + q21;
    return (q1 - q2) * normalized_xxx * 2. + (q2 - q1) * normalized_xx * 3. +
           q1;
  }

  const chisel::ChiselPtr getTSDF() const {
      return tsdf_;
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, const T& z,
                                      double* x1, double* y1, double* z1,
                                      double* x2, double* y2,
                                      double* z2, int coarsening_factor) const {
    const Eigen::Vector3f lower = CenterOfLowerVoxel(x, y, z, coarsening_factor);
    const chisel::ChunkManager& chunk_manager = tsdf_->GetChunkManager();
    const float resolution = chunk_manager.GetResolution()*coarsening_factor;
    *x1 = lower.x();
    *y1 = lower.y();
    *z1 = lower.z();
    *x2 = lower.x() + resolution;
    *y2 = lower.y() + resolution;
    *z2 = lower.z() + resolution;
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y, z). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector3f CenterOfLowerVoxel(const double x, const double y,
                                     const double z, int coarsening_factor) const {
      const chisel::ChunkManager& chunk_manager = tsdf_->GetChunkManager();
      const float round = 1/(chunk_manager.GetResolution()*coarsening_factor);
      const float x_0 = static_cast<float>(std::floor(x * round))/round;
      const float y_0 = static_cast<float>(std::floor(y * round))/round;
      const float z_0 = static_cast<float>(std::floor(z * round ))/round;
      Eigen::Vector3f center(x_0, y_0, z_0);
      return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector3f CenterOfLowerVoxel(const T& jet_x, const T& jet_y,
                                     const T& jet_z, int coarsening_factor) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a, jet_z.a, coarsening_factor);
  }


  double getVoxelSDF(const chisel::DistVoxel* voxel) const
  {
      double q = 0.5; //todo how to set value outside of tsdf
      if(voxel) {
        if(voxel->IsValid()) {
            q = voxel->GetSDF();
        }
      }
      return q;
  }

  const chisel::ChiselPtr tsdf_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_TSDF_H_
