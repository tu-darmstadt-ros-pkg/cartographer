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

#include "cartographer/mapping_3d/voxblox_tsdfs.h"

#include <cmath>
#include <limits>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include "cartographer/common/math.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping_3d/proto/tsdfs_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

//namespace {
proto::ProjectionIntegratorOptions CreateVoxbloxProjectionIntegratorOptions(
    common::LuaParameterDictionary* parameter_dictionary){
  proto::ProjectionIntegratorOptions options;
  options.set_truncation_scale(
      parameter_dictionary->GetDouble("truncation_scale"));
  options.set_truncation_distance(
      parameter_dictionary->GetDouble("truncation_distance"));
  options.set_carving_enabled(
      parameter_dictionary->GetBool("carving_enabled"));
  options.set_carving_distance(
      parameter_dictionary->GetDouble("carving_distance"));
  return options;
}


proto::TSDFsOptions CreateVoxbloxTSDFsOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::TSDFsOptions options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  options.set_chuck_size_x(
      parameter_dictionary->GetNonNegativeInt("chuck_size_x"));
  options.set_chuck_size_y(
      parameter_dictionary->GetNonNegativeInt("chuck_size_y"));
  options.set_chuck_size_z(
      parameter_dictionary->GetNonNegativeInt("chuck_size_z"));
  *options.mutable_projection_integrator_options() =
      CreateVoxbloxProjectionIntegratorOptions(
          parameter_dictionary->GetDictionary("projection_integrator").get());
  CHECK_GT(options.num_range_data(), 0);
  CHECK_GT(options.chuck_size_x(), 0);
  CHECK_GT(options.chuck_size_y(), 0);
  CHECK_GT(options.chuck_size_z(), 0);
  return options;
}


VoxbloxTSDF::VoxbloxTSDF(const float high_resolution, const float low_resolution,
               const transform::Rigid3d& origin, const int begin_range_data_index,
               float max_truncation_distance, Eigen::Vector3i& chunk_size)
    : mapping::Submap(origin),
      max_truncation_distance(max_truncation_distance){
    voxblox::TsdfMap::Config config;
    config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(high_resolution);
    //config.tsdf_voxels_per_side = voxels_per_side;
    tsdf.reset(new voxblox::TsdfMap(config));
}


VoxbloxTSDFs::VoxbloxTSDFs()
{
  // We always want to have at least one tsdf which we can return,
  // and will create it at the origin in absence of a better choice.
  AddTSDF(transform::Rigid3d::Identity());
}


VoxbloxTSDFs::VoxbloxTSDFs(const proto::TSDFsOptions& options)
    : options_(options) {
  // We always want to have at least one tsdf which we can return,
  // and will create it at the origin in absence of a better choice.
  AddTSDF(transform::Rigid3d::Identity());
}

const VoxbloxTSDF* VoxbloxTSDFs::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

const std::shared_ptr<voxblox::TsdfMap> VoxbloxTSDFs::GetVoxbloxTSDFPtr(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, size());
    return submaps_[index]->tsdf;
}

int VoxbloxTSDFs::size() const { return submaps_.size(); }



const std::shared_ptr<voxblox::TsdfIntegratorBase> VoxbloxTSDFs::GetIntegrator(int index) const{
    CHECK_GE(index, 0);
    CHECK_LT(index, size());
    return projection_integrators_[index];
  }


std::vector<int> VoxbloxTSDFs::insertion_indices() const {
  if (size() > 1) {
    return {size() - 2, size() - 1};
  }
  return {size() - 1};
}
void VoxbloxTSDFs::InsertRangeData(const sensor::RangeData& range_data_in_tracking,
                            const Eigen::Quaterniond& gravity_alignment,
                            const transform::Rigid3f &world_to_sensor)
{
    CHECK_LT(num_range_data_, std::numeric_limits<int>::max());
    ++num_range_data_;
    for(int index : insertion_indices()) {
      VoxbloxTSDF* submap = submaps_[index].get();
      const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
          transformed_range_data, submap->local_pose.inverse().cast<float>());

      voxblox::Transformation T_G_C;
      voxblox::Pointcloud points_C;
      voxblox::Colors colors;
      points_C.reserve(transformed_range_data.returns.size());
      colors.reserve(transformed_range_data.returns.size());

      for (const Eigen::Vector3f& pt : transformed_range_data.returns)
      {
          points_C.push_back(voxblox::Point(pt(0),
                                   pt(1),
                                   pt(2)));
          colors.push_back(
              voxblox::Color(128, 128, 128, 128)); //todo(kdaun) check where colors should come from
      }

      const transform::Rigid3f submap_to_sensor = submap->local_pose.inverse().cast<float>()*world_to_sensor;
      T_G_C.getPosition() = submap_to_sensor.translation();
      T_G_C.getRotation().setValues(submap_to_sensor.rotation().w(),
                                   submap_to_sensor.rotation().x(),
                                   submap_to_sensor.rotation().y(),
                                   submap_to_sensor.rotation().z());

      std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_ =
              projection_integrators_[index];

      tsdf_integrator_->integratePointCloud(T_G_C, points_C, colors);
    }


    for (const int index : insertion_indices()) {
      VoxbloxTSDF* submap = submaps_[index].get();
      ++submap->num_range_data;
    }

    ++num_range_data_in_last_submap_;
    if (num_range_data_in_last_submap_ == options_.num_range_data()) {
      AddTSDF(transform::Rigid3d(range_data_in_tracking.origin.cast<double>(),
                                 gravity_alignment));
    }
}


std::vector<Eigen::Array4i> VoxbloxTSDFs::ExtractVoxelData(
    const std::shared_ptr<voxblox::TsdfMap> hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) const {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution = hybrid_grid->getTsdfLayer().voxel_size();
  const float resolution_inverse = 1. / resolution;
  float min_sdf = -0.2f;
  float max_sdf = 0.2f;

  voxblox::BlockIndexList blocks;
  hybrid_grid->getTsdfLayer().getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = hybrid_grid->getTsdfLayer().voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double intensity = 0.0;
  // Iterate over all blocks.
  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = hybrid_grid->getTsdfLayer().getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      if (voxel.weight > 0.1) { // valid
        const float sdf = voxel.distance;
        int cell_value = common::RoundToInt((sdf - min_sdf) *
            (255.f / (max_sdf - min_sdf)));
        const Eigen::Vector3f cell_center_local =  Eigen::Vector3f(coord);
        const Eigen::Vector3f cell_center_global = transform * cell_center_local +Eigen::Vector3f({1e-5,1e-5,1e-5}); //todo(kdaun) handle cell offset between voxblox and cartographer grid
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
  return voxel_indices_and_probabilities;
}

string VoxbloxTSDFs::ComputePixelValues(
    const std::vector<VoxbloxTSDFs::PixelData>& accumulated_pixel_data) const {
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
      else if(pixel.count > 5) cell_value = 0;
      else cell_value = 100;
      cell_data.push_back(cell_value);  // value
      cell_data.push_back(90);  // alpha
  }
  return cell_data;
}

std::vector<VoxbloxTSDFs::PixelData> VoxbloxTSDFs::AccumulatePixelData(
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


void VoxbloxTSDFs::SubmapToProto(
    int index, const transform::Rigid3d& global_submap_pose,
    mapping::proto::SubmapQuery::Response* const response) const {
    // Generate an X-ray view through the 'hybrid_grid', aligned to the xy-plane
    // in the global map frame.
    const std::shared_ptr<voxblox::TsdfMap> hybrid_grid = Get(index)->tsdf;
    response->set_resolution(hybrid_grid->getTsdfLayer().voxel_size());

    // Compute a bounding box for the texture.
    Eigen::Array2i min_index(INT_MAX, INT_MAX);
    Eigen::Array2i max_index(INT_MIN, INT_MIN);
    const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
        ExtractVoxelData(hybrid_grid, global_submap_pose.cast<float>(),
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
                               max_index.x() * hybrid_grid->getTsdfLayer().voxel_size(),
                               max_index.y() * hybrid_grid->getTsdfLayer().voxel_size(),
                               global_submap_pose.translation().z())));
}


void VoxbloxTSDFs::AddTSDF(const transform::Rigid3d& origin) {
  if (size() > 1) {
    VoxbloxTSDF* submap = submaps_[size() - 2].get();
    CHECK(!submap->finished);
    submap->finished = true;
  }
  LOG(INFO)<<"TSDF origin: "<<origin;
  LOG(INFO)<<"Truncation distance: "<<options_.projection_integrator_options().truncation_distance();
//  double resolution = options_.high_resolution();
//  double truncation_distance = options_.projection_integrator_options().truncation_distance();
 // double truncation_scale = options_.projection_integrator_options().truncation_scale();
 // float max_truncation_distance = 0.15;//std::ceil(2.0 + (truncation_scale * truncation_distance /
                                         //   resolution)) * resolution;
  Eigen::Vector3i chunk_size;
  chunk_size.x() = options_.chuck_size_x();
  chunk_size.y() = options_.chuck_size_y();
  chunk_size.z() = options_.chuck_size_z();
  submaps_.emplace_back(new VoxbloxTSDF(options_.high_resolution(), options_.low_resolution(), origin,
                                   num_range_data_,options_.projection_integrator_options().truncation_distance(), chunk_size));
  /*chisel::ProjectionIntegrator projection_integrator;
  projection_integrator.SetCentroids(submaps_[size()-1]->tsdf->GetChunkManager().GetCentroids());
  projection_integrator.SetTruncator(chisel::TruncatorPtr(new chisel::ConstantTruncator(truncation_distance, truncation_scale)));
  projection_integrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(1)));
  projection_integrator.SetCarvingDist(options_.projection_integrator_options().carving_distance());
  projection_integrator.SetCarvingEnabled(options_.projection_integrator_options().carving_enabled());
  projection_integrators_.emplace_back(projection_integrator);*/


  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = options_.projection_integrator_options().truncation_distance();//config.tsdf_voxel_size * 2;
  integrator_config.max_ray_length_m = 30.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
  tsdf_integrator_.reset(new voxblox::SimpleTsdfIntegrator(
      integrator_config, submaps_[size()-1]->tsdf->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators
  projection_integrators_.emplace_back(tsdf_integrator_);

  /*
  LOG(INFO) << "truncation_distance " << truncation_distance<<" "<< truncation_scale;
  LOG(INFO) << "carving enabled " << options_.projection_integrator_options().carving_enabled();
  LOG(INFO) << "carving distance " << options_.projection_integrator_options().carving_distance();
  LOG(INFO) << "Added submap " << size();*/
  num_range_data_in_last_submap_ = 0;
}


}  // namespace mapping_3d
}  // namespace cartographer
