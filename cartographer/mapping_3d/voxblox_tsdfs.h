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

#ifndef CARTOGRAPHER_MAPPING_3D_VOXBLOX_TSDFS_H_
#define CARTOGRAPHER_MAPPING_3D_VOXBLOX_TSDFS_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/proto/tsdfs_options.pb.h"
#include "cartographer/mapping_3d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform.h"

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace cartographer {
namespace mapping_3d {

proto::ProjectionIntegratorOptions CreateProjectionIntegratorOptions(
    common::LuaParameterDictionary* parameter_dictionary);

proto::TSDFsOptions CreateTSDFsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

struct VoxbloxTSDF : public mapping::Submap {
  VoxbloxTSDF(float high_resolution, float low_resolution,
         const transform::Rigid3d& origin, int begin_range_data_index,
         float max_truncation_distance, Eigen::Vector3i& chunk_size);

  std::shared_ptr<voxblox::TsdfMap> tsdf;
  bool finished = false;
  float max_truncation_distance;
  std::vector<int> trajectory_node_indices;
};


// A container for Truncated Signed Distance Fields similar to the Submaps container.
class VoxbloxTSDFs : public mapping::Submaps {
 public:
  VoxbloxTSDFs();
  VoxbloxTSDFs(const proto::TSDFsOptions& options);

  VoxbloxTSDFs(const VoxbloxTSDFs&) = delete;
  VoxbloxTSDFs& operator=(const VoxbloxTSDFs&) = delete;

  const VoxbloxTSDF* Get(int index) const override;
  const chisel::ChiselPtr<chisel::DistVoxel> GetChiselPtr(int index) const override{
      LOG(FATAL) << "Not implemented."; }
  const std::shared_ptr<voxblox::TsdfMap> GetVoxbloxTSDFPtr(int index) const override;
  const std::shared_ptr<voxblox::TsdfIntegratorBase> GetIntegrator(int index) const;
  int size() const override;

    // Returns the indices of the Submap into which point clouds will
  // be inserted.
  std::vector<int> insertion_indices() const;
  //Inserts 'range_data' into the Submap collection.
  void InsertRangeData(const sensor::RangeData& range_data_in_tracking,
                       const Eigen::Quaterniond &gravity_alignment,
                       const Eigen::Vector3f& sensor_origin);



  void SubmapToProto(
      int index,
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;

  /*

  // Returns the 'high_resolution' HybridGrid to be used for matching.
  const HybridGrid& high_resolution_matching_grid() const;

  // Returns the 'low_resolution' HybridGrid to be used for matching.
  const HybridGrid& low_resolution_matching_grid() const;*/

 private:
  struct PixelData {
    int min_z = INT_MAX;
    int max_z = INT_MIN;
    int count = 0;
    float probability_sum = 0.f;
    float max_probability = 0.5f;
  };

  std::vector<PixelData> AccumulatePixelData(
      const int width, const int height, const Eigen::Array2i& min_index,
      const Eigen::Array2i& max_index,
      const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) const;
  // The first three entries of each returned value are a cell_index and the
  // last is the corresponding probability value. We batch them together like
  // this to only have one vector and have better cache locality.
  std::vector<Eigen::Array4i> ExtractVoxelData(
      const std::shared_ptr<voxblox::TsdfMap> hybrid_grid, const transform::Rigid3f& transform,
      Eigen::Array2i* min_index, Eigen::Array2i* max_index) const;
  // Builds texture data containing interleaved value and alpha for the
  // visualization from 'accumulated_pixel_data'.
  string ComputePixelValues(
      const std::vector<PixelData>& accumulated_pixel_data) const;

  void AddTSDF(const transform::Rigid3d &origin);

  const proto::TSDFsOptions options_;

  std::vector<std::unique_ptr<VoxbloxTSDF>> submaps_;
  std::vector<std::shared_ptr<voxblox::TsdfIntegratorBase>> projection_integrators_;

  // Number of RangeData inserted.
  int num_range_data_ = 0;

  // Number of RangeData inserted since the last Submap was added.
  int num_range_data_in_last_submap_ = 0;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_VOXBLOX_TSDFS_H_
