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
#include <open_chisel/truncation/QuadraticTruncator.h>
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
               const Eigen::Vector3f& origin, const int begin_range_data_index)
    : mapping::Submap(origin, begin_range_data_index) {
    tsdf.reset(new chisel::Chisel(Eigen::Vector3i(16, 16, 16), 0.05, false, origin));
    //todo load params from config
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

const chisel::ChiselPtr TSDFs::GetChiselPtr(int index) const {
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
        chisel::ChiselPtr chisel_tsdf = submaps_[insertion_index]->tsdf;
        const chisel::ProjectionIntegrator& projection_integrator =
                projection_integrators_[insertion_index];
        chisel_tsdf->GetMutableChunkManager().clearIncrementalChanges();
        //todo transform data before to avoid double transformation
        chisel_tsdf->IntegratePointCloud(projection_integrator, cloudOut,
                                         transform, chisel_pose, 0.5f, 6.f);
        //TODO  set far/near plane in config
        chisel_tsdf->UpdateMeshes();
    }

    ++num_range_data_in_last_submap_;
    if (num_range_data_in_last_submap_ == options_.num_range_data()) {
      AddTSDF(range_data_in_tracking.origin);
    }
}



void TSDFs::SubmapToProto(
    int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
    const transform::Rigid3d& global_submap_pose,
    mapping::proto::SubmapQuery::Response* const response) const {
    LOG(WARNING) << "SubmapToProto not implemented for TSDF ";
    //TODO implement
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
                                   num_range_data_));
  chisel::ProjectionIntegrator projection_integrator;
  projection_integrator.SetCentroids(submaps_[size()-1]->tsdf->GetChunkManager().GetCentroids());
  projection_integrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(0.0, 0.0, 0.01, 16.0)));
  projection_integrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(1)));
  projection_integrator.SetCarvingDist(0.1);
  projection_integrator.SetCarvingEnabled(false);
  projection_integrators_.emplace_back(projection_integrator);

  LOG(INFO) << "Added submap " << size();
  num_range_data_in_last_submap_ = 0;
}


}  // namespace mapping_3d
}  // namespace cartographer
