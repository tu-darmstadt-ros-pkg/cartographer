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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_TSDF_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_TSDF_H_

#include <cmath>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>

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



class InterpolatedVoxbloxTSDF {
public:
  explicit InterpolatedVoxbloxTSDF(const std::shared_ptr<voxblox::TsdfMap>  tsdf,
                                   float max_truncation_distance,
                                   bool use_cubic_interpolation = true,
                                   bool use_boundary_extrapolation = true)
    : tsdf_(tsdf), max_truncation_distance_(max_truncation_distance),
      use_cubic_interpolation_(use_cubic_interpolation),
      use_boundary_extrapolation_(use_boundary_extrapolation) {}

  InterpolatedVoxbloxTSDF(const InterpolatedVoxbloxTSDF&) = delete;
  InterpolatedVoxbloxTSDF& operator=(const InterpolatedVoxbloxTSDF&) = delete;

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
    //const auto& chunk_manager = tsdf_->GetChunkManager(); //todo(kdaun) reenable
    //chisel::Vec3 origin = chunk_manager.GetOrigin();
    T x_local = x;// - T(origin.x());
    T y_local = y;// - T(origin.y());
    T z_local = z;// - T(origin.z());

    ComputeInterpolationDataPoints(x_local, y_local, z_local, &x1, &y1, &z1, &x2, &y2, &z2, coarsening_factor);

    double q111, q112, q121, q122, q211, q212, q221, q222;
    size_t num_invalid_voxel = 0;
    double summed_valid_sdf = 0.0;
    if(!getVoxelSDF(x1, y1, z1, q111)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q111;
    }
    if(!getVoxelSDF(x1, y1, z2, q112)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q112;
    }
    if(!getVoxelSDF(x1, y2, z1, q121)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q121;
    }
    if(!getVoxelSDF(x1, y2, z2, q122)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q122;
    }
    if(!getVoxelSDF(x2, y1, z1, q211)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q211;
    }
    if(!getVoxelSDF(x2, y1, z2, q212)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q212;
    }
    if(!getVoxelSDF(x2, y2, z1, q221)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q221;
    }
    if(!getVoxelSDF(x2, y2, z2, q222)) {
      num_invalid_voxel++;
    }
    else
    {
      summed_valid_sdf += q222;
    }

    if(num_invalid_voxel > 0)
    {
      double signed_max_tsdf = summed_valid_sdf < 0 ? - max_truncation_distance_ : max_truncation_distance_;
      if(use_boundary_extrapolation_) {
        if(std::isnan(q111)) {
          q111 = signed_max_tsdf;
        }
        if(std::isnan(q112)) {
          q112 = signed_max_tsdf;
        }
        if(std::isnan(q121)) {
          q121 = signed_max_tsdf;
        }
        if(std::isnan(q122)) {
          q122 = signed_max_tsdf;
        }
        if(std::isnan(q211)) {
          q211 = signed_max_tsdf;
        }
        if(std::isnan(q212)) {
          q212 = signed_max_tsdf;
        }
        if(std::isnan(q221)) {
          q221 = signed_max_tsdf;
        }
        if(std::isnan(q222)) {
          q222 = signed_max_tsdf;
        }
      }
      else {
        q111 = max_truncation_distance_;
        q112 = max_truncation_distance_;
        q121 = max_truncation_distance_;
        q122 = max_truncation_distance_;
        q211 = max_truncation_distance_;
        q212 = max_truncation_distance_;
        q221 = max_truncation_distance_;
        q222 = max_truncation_distance_;
      }
    }

    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);
    const T normalized_z = (z - z1) / (z2 - z1);

    if(use_cubic_interpolation_) {
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
    else { //trilinear interpolation https://en.wikipedia.org/wiki/Trilinear_interpolation
      const T c00 = q111 * (T(1.0) - normalized_x) + q211 * normalized_x;
      const T c01 = q112 * (T(1.0) - normalized_x) + q212 * normalized_x;
      const T c10 = q121 * (T(1.0) - normalized_x) + q221 * normalized_x;
      const T c11 = q122 * (T(1.0) - normalized_x) + q222 * normalized_x;

      const T c0 = c00 * (T(1.0) - normalized_y) + c10 * normalized_y;
      const T c1 = c01 * (T(1.0) - normalized_y) + c11 * normalized_y;

      const T c = c0 * (T(1.0) - normalized_z) + c1 * normalized_z;
      return c;
    }
  }

  const std::shared_ptr<voxblox::TsdfMap> getTSDF() const {
    return tsdf_;
  }

private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, const T& z,
                                      double* x1, double* y1, double* z1,
                                      double* x2, double* y2,
                                      double* z2, int coarsening_factor) const {
    const Eigen::Vector3f lower = CenterOfLowerVoxel(x, y, z, coarsening_factor);
    //const auto& chunk_manager = tsdf_->GetChunkManager();
    const float resolution = tsdf_->getTsdfLayer().voxel_size(); //todo(kdaun) handle different resolutions for diffrent directions
    //LOG(INFO)<<"resolution "<<resolution;
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
    const float coarsed_resolution = tsdf_->getTsdfLayer().voxel_size()*coarsening_factor;
    //LOG(INFO)<<"resolution "<< tsdf_->getTsdfLayer().voxel_size();
    const float round = 1/coarsed_resolution;
    const float offset = 0.5 * coarsed_resolution;
    float x_0 = static_cast<float>((std::floor(x * round + 0.5)/round) - offset);
    float y_0 = static_cast<float>((std::floor(y * round + 0.5)/round) - offset);
    float z_0 = static_cast<float>((std::floor(z * round + 0.5)/round) - offset);
    Eigen::Vector3f center(x_0, y_0, z_0);
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector3f CenterOfLowerVoxel(const T& jet_x, const T& jet_y,
                                     const T& jet_z, int coarsening_factor) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a, jet_z.a, coarsening_factor);
  }


  bool getVoxelSDF(double x, double y, double z, double& sdf) const
  {
    sdf = NAN;
    bool valid = false;
    voxblox::Point point(x, y, z);
    voxblox::Layer<voxblox::TsdfVoxel>::BlockType::ConstPtr block_ptr =
        tsdf_->getTsdfLayer().getBlockPtrByCoordinates(point);
    if (block_ptr != nullptr) {
      const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point);
      if (voxel.weight > 0.0)
      {
        sdf = (double)voxel.distance;
        valid = true;
      }
      if(std::abs(sdf) > max_truncation_distance_ )
      {
        //LOG(WARNING)<<"q >>> max_truncation_distance "<< q <<" > "<< max_truncation_distance_;
        //LOG(WARNING)<<"weight: "<< voxel.weight; //todo(kdaun) enable warnings
        sdf = NAN;
        valid = false;
      }
    }
    return valid;
  }

  const std::shared_ptr<voxblox::TsdfMap> tsdf_;
  float max_truncation_distance_;
  const bool use_cubic_interpolation_;
  const bool use_boundary_extrapolation_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_TSDF_H_
