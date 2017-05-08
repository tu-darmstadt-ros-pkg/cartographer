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

#include "cartographer/mapping_3d/scan_matching/precomputation_grid.h"

#include <algorithm>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"
#include <open_chisel/Chisel.h>
#include <open_chisel/MultiDistVoxel.h>

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

namespace {

// C++11 defines that integer division rounds towards zero. For index math, we
// actually need it to round towards negative infinity. Luckily bit shifts have
// that property.
inline int DivideByTwoRoundingTowardsNegativeInfinity(const int value) {
  return value >> 1;
}

// Computes the half resolution index corresponding to the full resolution
// 'cell_index'.
Eigen::Array3i CellIndexAtHalfResolution(const Eigen::Array3i& cell_index) {
  return Eigen::Array3i(
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[0]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[1]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[2]));
}

}  // namespace

PrecomputationGrid ConvertToPrecomputationGrid(const chisel::ChiselPtr<chisel::MultiDistVoxel> hybrid_grid) {
  PrecomputationGrid result(hybrid_grid->GetChunkManager().GetResolution(), hybrid_grid->GetChunkManager().GetOrigin());
 /* for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const int cell_value = common::RoundToInt(
        (mapping::ValueToProbability(it.GetValue()) -
         mapping::kMinProbability) *
        (255.f / (mapping::kMaxProbability - mapping::kMinProbability)));
    CHECK_GE(cell_value, 0);
    CHECK_LE(cell_value, 255);
    *result.mutable_value(it.GetCellIndex()) = cell_value;
  }*/ //todo(kdaun) implement iterator and conversion for tsdf
  const chisel::AABB& bounding_box = hybrid_grid->GetChunkManager().GetBoundingBox();
  const chisel::Vec3& min = bounding_box.min;
  const chisel::Vec3& max = bounding_box.max;
  Eigen::Array3f origin = hybrid_grid->GetChunkManager().GetOrigin();
  float resolution = hybrid_grid->GetChunkManager().GetResolution();
  float min_sdf = 0.f;
  float max_sdf = 0.5f;
  //float max_extension = std::max(std::max(max.x() - min.x(), max.y() - min.y()), max.z() - min.z());
  //int bits = common::RoundToInt(std::log2(max_extension/min_sdf));

  for(float x = min.x(); x < max.x(); x = x + resolution)
  {
      for(float y = min.y(); y < max.y(); y = y + resolution)
      {
          for(float z = min.z(); z < max.z(); z = z + resolution)
          {
              const auto& chunk_manager = hybrid_grid->GetChunkManager();
              const chisel::MultiDistVoxel* voxel = chunk_manager.GetDistanceVoxelGlobal(chisel::Vec3(x,y,z));
              if(voxel) {
                if(voxel->IsValid()) {
                    const float sdf = std::abs(voxel->GetSDF());
                    const int cell_value = 255 - common::RoundToInt(sdf - min_sdf) *
                        (255.f / (max_sdf - min_sdf));
                    Eigen::Array3f point;
                    point.x() = x;
                    point.y() = y;
                    point.z() = z;
                    Eigen::Array3f index_float = (point - origin).array() / resolution;
                    Eigen::Array3i index_int(common::RoundToInt(index_float.x()),
                                          common::RoundToInt(index_float.y()),
                                          common::RoundToInt(index_float.z()));
                    *result.mutable_value(index_int) = cell_value;
                }
              }
          }
      }
  }


  return result;
}

PrecomputationGrid ConvertToPrecomputationGrid(const HybridGrid& hybrid_grid) {
  PrecomputationGrid result(hybrid_grid.resolution(), hybrid_grid.origin());
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const int cell_value = common::RoundToInt(
        (mapping::ValueToProbability(it.GetValue()) -
         mapping::kMinProbability) *
        (255.f / (mapping::kMaxProbability - mapping::kMinProbability)));
    CHECK_GE(cell_value, 0);
    CHECK_LE(cell_value, 255);
    *result.mutable_value(it.GetCellIndex()) = cell_value;
  }
  return result;
}

PrecomputationGrid PrecomputeGrid(const PrecomputationGrid& grid,
                                  const bool half_resolution,
                                  const Eigen::Array3i& shift) {
  PrecomputationGrid result(grid.resolution(), grid.origin());
  for (auto it = PrecomputationGrid::Iterator(grid); !it.Done(); it.Next()) {
    for (int i = 0; i != 8; ++i) {
      // We use this value to update 8 values in the resulting grid, at
      // position (x - {0, 'shift'}, y - {0, 'shift'}, z - {0, 'shift'}).
      // If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid,
      // this results in precomputation grids analogous to the 2D case.
      const Eigen::Array3i cell_index =
          it.GetCellIndex() - shift * PrecomputationGrid::GetOctant(i);
      auto* const cell_value = result.mutable_value(
          half_resolution ? CellIndexAtHalfResolution(cell_index) : cell_index);
      *cell_value = std::max(it.GetValue(), *cell_value);
    }
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
