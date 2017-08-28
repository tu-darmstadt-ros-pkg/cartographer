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

#include "cartographer/mapping/map_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/collated_trajectory_builder.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/global_tsdf_trajectory_builder.h"
#include "cartographer/mapping_3d/global_voxblox_tsdf_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  *options.mutable_sparse_pose_graph_options() = CreateSparsePoseGraphOptions(
      parameter_dictionary->GetDictionary("sparse_pose_graph").get());
  const string map_type_string = parameter_dictionary->GetString("map_type");
  proto::MapBuilderOptions::MapType map_type;
  CHECK(proto::MapBuilderOptions::MapType_Parse(map_type_string, &map_type))
      << "Unknown MapBuilderOptions kind: " << map_type_string;
  options.set_map_type(map_type);
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
  : options_(options), thread_pool_(options.num_background_threads()) {
  if (options.use_trajectory_builder_2d()) {
    sparse_pose_graph_2d_ = common::make_unique<mapping_2d::SparsePoseGraph>(
          options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_2d_.get();
  }
  if (options.use_trajectory_builder_3d()) {
    switch (options_.map_type()) {
    case proto::MapBuilderOptions::PROBABILITY_GRID:
      LOG(INFO)<<"MAP_TYPE: PROBABILITY_GRID";
      sparse_pose_graph_3d_ = common::make_unique<mapping_3d::SparsePoseGraph>(
            options_.sparse_pose_graph_options(), &thread_pool_);
      sparse_pose_graph_ = sparse_pose_graph_3d_.get();
      break;
    case proto::MapBuilderOptions::CHISEL_TSDF:
      LOG(INFO)<<"MAP_TYPE: CHISEL_TSDF";
      sparse_pose_graph_tsdf_3d_ = common::make_unique<mapping_3d::SparsePoseGraphConversion>(
            options_.sparse_pose_graph_options(), &thread_pool_);
      sparse_pose_graph_ = sparse_pose_graph_tsdf_3d_.get();
      break;
    case proto::MapBuilderOptions::VOXBLOX_TSDF:
      LOG(INFO)<<"MAP_TYPE: VOXBLOX_TSDF";
      sparse_pose_graph_voxblox_tsdf_3d_ = common::make_unique<mapping_3d::SparsePoseGraphVoxbloxConversion>(
            options_.sparse_pose_graph_options(), &thread_pool_);
      sparse_pose_graph_ = sparse_pose_graph_voxblox_tsdf_3d_.get();
      break;
    case proto::MapBuilderOptions::VOXBLOX_ESDF:
      LOG(INFO)<<"MAP_TYPE: VOXBLOX_ESDF";
      LOG(FATAL)<<"MapBuilderOptions::VOXBLOX_ESDF not implemented";
      break;
    }
  }
}

MapBuilder::~MapBuilder() {}

int MapBuilder::AddTrajectoryBuilder(
    const std::unordered_set<string>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options) {
  const int trajectory_id = trajectory_builders_.size();
  if (options_.use_trajectory_builder_3d()) {
    CHECK(trajectory_options.has_trajectory_builder_3d_options());
    switch (options_.map_type()) {
    case proto::MapBuilderOptions::PROBABILITY_GRID:
      trajectory_builders_.push_back(
            common::make_unique<CollatedTrajectoryBuilder>(
              &sensor_collator_, trajectory_id, expected_sensor_ids,
              common::make_unique<mapping_3d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_3d_.get())));
      break;
    case proto::MapBuilderOptions::CHISEL_TSDF:
      trajectory_builders_.push_back(
            common::make_unique<CollatedTrajectoryBuilder>(
              &sensor_collator_, trajectory_id, expected_sensor_ids,
              common::make_unique<mapping_3d::GlobalTSDFTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_tsdf_3d_.get())));
      break;
    case proto::MapBuilderOptions::VOXBLOX_TSDF:
      trajectory_builders_.push_back(
            common::make_unique<CollatedTrajectoryBuilder>(
              &sensor_collator_, trajectory_id, expected_sensor_ids,
              common::make_unique<mapping_3d::GlobalVoxbloxTSDFTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_voxblox_tsdf_3d_.get())));
      break;
    case proto::MapBuilderOptions::VOXBLOX_ESDF:
      LOG(FATAL)<<"MapBuilderOptions::VOXBLOX_ESDF not implemented";
      break;
    }
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_2d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_2d_options(),
                trajectory_id, sparse_pose_graph_2d_.get())));
  }
  return trajectory_id;
}

TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}

int MapBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}


proto::TrajectoryConnectivity MapBuilder::GetTrajectoryConnectivity() {
  return ToProto(sparse_pose_graph_->GetConnectedTrajectories());
}

string MapBuilder::SubmapToProto(const int trajectory_id,
                                 const int submap_index,
                                 proto::SubmapQuery::Response* const response) {
    if (trajectory_id < 0 || trajectory_id >= num_trajectory_builders()) {
      return "Requested submap from trajectory " + std::to_string(trajectory_id) +
             " but there are only " + std::to_string(num_trajectory_builders()) +
             " trajectories.";
    }

    const std::vector<transform::Rigid3d> submap_transforms =
        sparse_pose_graph_->GetSubmapTransforms(trajectory_id);
    if (submap_index < 0 ||
        static_cast<size_t>(submap_index) >= submap_transforms.size()) {
      return "Requested submap " + std::to_string(submap_index) +
             " from trajectory " + std::to_string(trajectory_id) +
             " but there are only " + std::to_string(submap_transforms.size()) +
             " submaps in this trajectory.";
    }

    const Submaps* const submaps =
        trajectory_builders_.at(trajectory_id)->submaps();
    response->set_submap_version(submaps->Get(submap_index)->num_range_data);
    submaps->SubmapToProto(submap_index, submap_transforms[submap_index],
                           response);
    return "";
  }


int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

SparsePoseGraph* MapBuilder::sparse_pose_graph() { return sparse_pose_graph_; }

void MapBuilder::reset() {

    if (options_.use_trajectory_builder_2d()) {
      sparse_pose_graph_2d_ = common::make_unique<mapping_2d::SparsePoseGraph>(
          options_.sparse_pose_graph_options(), &thread_pool_);
      sparse_pose_graph_ = sparse_pose_graph_2d_.get();
    }
    if (options_.use_trajectory_builder_3d()) {
        switch (options_.map_type()) {
        case proto::MapBuilderOptions::PROBABILITY_GRID:
          sparse_pose_graph_3d_ = common::make_unique<mapping_3d::SparsePoseGraph>(
                options_.sparse_pose_graph_options(), &thread_pool_);
          sparse_pose_graph_ = sparse_pose_graph_3d_.get();
          break;
        case proto::MapBuilderOptions::CHISEL_TSDF:
          sparse_pose_graph_tsdf_3d_ = common::make_unique<mapping_3d::SparsePoseGraphConversion>(
                options_.sparse_pose_graph_options(), &thread_pool_);
          sparse_pose_graph_ = sparse_pose_graph_tsdf_3d_.get();
          break;
        case proto::MapBuilderOptions::VOXBLOX_TSDF:
          sparse_pose_graph_voxblox_tsdf_3d_ = common::make_unique<mapping_3d::SparsePoseGraphVoxbloxConversion>(
                options_.sparse_pose_graph_options(), &thread_pool_);
          sparse_pose_graph_ = sparse_pose_graph_voxblox_tsdf_3d_.get();
          break;
        case proto::MapBuilderOptions::VOXBLOX_ESDF:
          LOG(FATAL)<<"MapBuilderOptions::VOXBLOX_ESDF not implemented";
          break;
        }
    }
}

}  // namespace mapping
}  // namespace cartographer
