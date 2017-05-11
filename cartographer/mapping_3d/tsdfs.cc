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

#include "cartographer/mapping_3d/tsdfs.h"

#include <cmath>
#include <limits>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include "cartographer/common/math.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

//namespace {
/*
proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}
*/

TSDF::TSDF(const float high_resolution, const float low_resolution,
               const Eigen::Vector3f& origin, const int begin_range_data_index,
               float max_truncation_distance)
    : mapping::Submap(origin, begin_range_data_index),
    max_truncation_distance(max_truncation_distance){
    tsdf.reset(new chisel::Chisel<chisel::MultiDistVoxel>
               (Eigen::Vector3i(16, 16, 16), 0.05, false, origin));
    //todo(kdaun) load params from config
}

TSDFs::TSDFs()
{
  // We always want to have at least one tsdf which we can return,
  // and will create it at the origin in absence of a better choice.
  AddTSDF(Eigen::Vector3f::Zero());
}


TSDFs::TSDFs(const proto::SubmapsOptions& options)
    : options_(options) {
  // We always want to have at least one tsdf which we can return,
  // and will create it at the origin in absence of a better choice.
  AddTSDF(Eigen::Vector3f::Zero());
}

const TSDF* TSDFs::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

const chisel::ChiselPtr<chisel::MultiDistVoxel> TSDFs::GetChiselPtr(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, size());
    return submaps_[index]->tsdf;
}

int TSDFs::size() const { return submaps_.size(); }



const chisel::ProjectionIntegrator* TSDFs::GetIntegrator(int index) const{
    CHECK_GE(index, 0);
    CHECK_LT(index, size());
    return &projection_integrators_[index];
  }


std::vector<int> TSDFs::insertion_indices() const {
  if (size() > 1) {
    return {size() - 2, size() - 1};
  }
  return {size() - 1};
}
void TSDFs::InsertRangeData(const sensor::RangeData& range_data_in_tracking,
                            const transform::Rigid3d& pose_observation)
{
    CHECK_LT(num_range_data_, std::numeric_limits<int>::max());
    ++num_range_data_;

    chisel::PointCloud cloudOut;
    cloudOut.GetMutablePoints().resize(range_data_in_tracking.returns.size());

    size_t i = 0;
    for (const Eigen::Vector3f& pt : range_data_in_tracking.returns)
    {
        chisel::Vec3& xyz =  cloudOut.GetMutablePoints().at(i);
        xyz(0) = pt(0);
        xyz(1) = pt(1);
        xyz(2) = pt(2);
        i++;
    }

    chisel::Transform transform;
    transform.translation()(0) = pose_observation.translation()(0);
    transform.translation()(1) = pose_observation.translation()(1);
    transform.translation()(2) = pose_observation.translation()(2);


    chisel::Vec3 chisel_pose;
    chisel_pose.x() = pose_observation.translation()(0);
    chisel_pose.y() = pose_observation.translation()(1);
    chisel_pose.z() = pose_observation.translation()(2) + 0.5;


    chisel::Quaternion quat;
    quat.x() = pose_observation.rotation().x();
    quat.y() = pose_observation.rotation().y();
    quat.z() = pose_observation.rotation().z();
    quat.w() = pose_observation.rotation().w();
    transform.linear() = quat.toRotationMatrix();


    //std::vector<int> insertion_indices = insertion_indices();
    for(int insertion_index : insertion_indices())
    {
        chisel::ChiselPtr<chisel::MultiDistVoxel> chisel_tsdf = submaps_[insertion_index]->tsdf;
        TSDF* submap = submaps_[insertion_index].get();
        const chisel::ProjectionIntegrator& projection_integrator =
                projection_integrators_[insertion_index];
        chisel_tsdf->GetMutableChunkManager().clearIncrementalChanges();
        //todo transform data before to avoid double transformation
        chisel_tsdf->IntegratePointCloud(projection_integrator, cloudOut,
                                         transform, chisel_pose, 0.5f, 6.f);
        //TODO  set far/near plane in config
        chisel_tsdf->UpdateMeshes();
        submap->end_range_data_index = num_range_data_;
    }

    ++num_range_data_in_last_submap_;
    if (num_range_data_in_last_submap_ == options_.num_range_data()) {
      AddTSDF(range_data_in_tracking.origin);
    }
}



std::vector<Eigen::Array4i> TSDFs::ExtractVoxelData(
    const chisel::ChiselPtr<chisel::MultiDistVoxel> hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) const {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution = hybrid_grid->GetChunkManager().GetResolution();
  const float resolution_inverse = 1. / hybrid_grid->GetChunkManager().GetResolution();
  const chisel::AABB& bounding_box = hybrid_grid->GetChunkManager().GetBoundingBox();
  const chisel::Vec3& min = bounding_box.min;
  const chisel::Vec3& max = bounding_box.max;
  float min_sdf = -0.4f;
  float max_sdf = 0.4f;
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
                    const float sdf = voxel->GetSDF();
                    int cell_value = common::RoundToInt((sdf - min_sdf) *
                        (255.f / (max_sdf - min_sdf)));
                    const Eigen::Vector3f cell_center_local =  Eigen::Vector3f(x,y,z);
                    const Eigen::Vector3f cell_center_global = transform * cell_center_local;
                    const Eigen::Array4i voxel_index_and_probability(
                        common::RoundToInt(cell_center_global.x() * resolution_inverse),
                        common::RoundToInt(cell_center_global.y() * resolution_inverse),
                        common::RoundToInt(cell_center_global.z() * resolution_inverse),
                        cell_value);

                    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
                    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
                    *min_index = min_index->cwiseMin(pixel_index);
                    *max_index = max_index->cwiseMax(pixel_index);
                }
              }
          }
      }
  }
  return voxel_indices_and_probabilities;
}

string TSDFs::ComputePixelValues(
    const std::vector<TSDFs::PixelData>& accumulated_pixel_data) const {
  string cell_data;
  cell_data.reserve(2 * accumulated_pixel_data.size());
  for (const PixelData& pixel : accumulated_pixel_data) {
      if(pixel.count == 0)
      {
          cell_data.push_back(0);  // value
          cell_data.push_back(0);  // alpha
          continue;
      }
      int cell_value = pixel.probability_sum/pixel.count;
      if(cell_value > 115) cell_value = 255; //todo(kdaun) replace by sound occlusion formulation
      else
          cell_value = 0;
      cell_data.push_back(cell_value);  // value
      cell_data.push_back(70);  // alpha
  }
  return cell_data;
}

std::vector<TSDFs::PixelData> TSDFs::AccumulatePixelData(
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) const {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability = voxel_index_and_probability[3];
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  return accumulated_pixel_data;
}


void TSDFs::SubmapToProto(
    int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
    const transform::Rigid3d& global_submap_pose,
    mapping::proto::SubmapQuery::Response* const response) const {
    // Generate an X-ray view through the 'hybrid_grid', aligned to the xy-plane
    // in the global map frame.
    const chisel::ChiselPtr<chisel::MultiDistVoxel> hybrid_grid = Get(index)->tsdf;
    response->set_resolution(hybrid_grid->GetChunkManager().GetResolution());

    // Compute a bounding box for the texture.
    Eigen::Array2i min_index(INT_MAX, INT_MAX);
    Eigen::Array2i max_index(INT_MIN, INT_MIN);
    const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
        ExtractVoxelData(hybrid_grid,
                         (global_submap_pose * Get(index)->local_pose().inverse())
                             .cast<float>(),
                         &min_index, &max_index);

    const int width = max_index.y() - min_index.y() + 1;
    const int height = max_index.x() - min_index.x() + 1;
    response->set_width(width);
    response->set_height(height);

    const std::vector<PixelData> accumulated_pixel_data = AccumulatePixelData(
        width, height, min_index, max_index, voxel_indices_and_probabilities);
    const string cell_data = ComputePixelValues(accumulated_pixel_data);

    common::FastGzipString(cell_data, response->mutable_cells());
    *response->mutable_slice_pose() =
        transform::ToProto(global_submap_pose.inverse() *
                           transform::Rigid3d::Translation(Eigen::Vector3d(
                               max_index.x() * hybrid_grid->GetChunkManager().GetResolution(),
                               max_index.y() * hybrid_grid->GetChunkManager().GetResolution(),
                               global_submap_pose.translation().z())));
}

void TSDFs::AddTrajectoryNodeIndex(const int trajectory_node_index) {
  for (int i = 0; i != size(); ++i) {
    TSDF& submap = *submaps_[i];
    if (submap.end_range_data_index == num_range_data_ &&
        submap.begin_range_data_index <= num_range_data_ - 1) {
      submap.trajectory_node_indices.push_back(trajectory_node_index);
    }
  }
}


/*
const HybridGrid& Submaps::high_resolution_matching_grid() const {
  return submaps_[matching_index()]->high_resolution_hybrid_grid;
}

const HybridGrid& Submaps::low_resolution_matching_grid() const {
  return submaps_[matching_index()]->low_resolution_hybrid_grid;
}
*/


void TSDFs::AddTSDF(const Eigen::Vector3f& origin) {
  if (size() > 1) {
    TSDF* submap = submaps_[size() - 2].get();
    CHECK(!submap->finished);
    submap->finished = true;
  }
  submaps_.emplace_back(new TSDF(options_.high_resolution(),
                                   options_.low_resolution(), origin,
                                   num_range_data_,(std::ceil(0.01*16.0/0.05)*0.05)));
  chisel::ProjectionIntegrator projection_integrator;
  projection_integrator.SetCentroids(submaps_[size()-1]->tsdf->GetChunkManager().GetCentroids());
  projection_integrator.SetTruncator(chisel::TruncatorPtr(new chisel::ConstantTruncator(0.01, 16.0)));
  //todo(kdaun) load params from config
  projection_integrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(1)));
  projection_integrator.SetCarvingDist(0.1);
  projection_integrator.SetCarvingEnabled(false);
  projection_integrators_.emplace_back(projection_integrator);

  LOG(INFO) << "Added submap " << size();
  num_range_data_in_last_submap_ = 0;
}


}  // namespace mapping_3d
}  // namespace cartographer
