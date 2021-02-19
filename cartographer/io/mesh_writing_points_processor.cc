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

#include "cartographer/io/mesh_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

#ifdef WITH_OPEN3D
#include "open3d/Open3D.h"
#endif

namespace cartographer {
  namespace io {

    std::unique_ptr<MeshWritingPointsProcessor>
    MeshWritingPointsProcessor::FromDictionary(
            const FileWriterFactory& file_writer_factory,
            common::LuaParameterDictionary* const dictionary,
            PointsProcessor* const next) {
      return absl::make_unique<MeshWritingPointsProcessor>(
              file_writer_factory(dictionary->GetString("filename")),
              dictionary->GetInt("aggregate"),
              dictionary->HasKey("poisson_depth") ? dictionary->GetInt("poisson_depth") : 0,
              dictionary->HasKey("trim_surface") ? dictionary->GetDouble("trim_surface") : 0,
              dictionary->HasKey("statistical_outlier_neighbours") ? dictionary->GetInt("statistical_outlier_neighbours") : 0,
              dictionary->HasKey("statistical_outlier_radius") ? dictionary->GetDouble("statistical_outlier_radius") : 0,
              std::vector<std::string>(), next);
    }

    MeshWritingPointsProcessor::MeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                                         const size_t &aggregate,
                                                         const int64 &poisson_depth,
                                                         const double &trim_surface,
                                                         const int64 &statistical_outlier_neighbours,
                                                         const double &statistical_outlier_radius,
                                                         const std::vector<std::string> &comments,
                                                         PointsProcessor *const next)
            : next_(next),
              aggregate_(aggregate),
              poisson_depth_(poisson_depth),
              trim_surface_(trim_surface),
              statistical_outlier_neighbours_(statistical_outlier_neighbours),
              statistical_outlier_radius_(statistical_outlier_radius),
              comments_(comments),
              num_points_(0),
              currentTime_(common::FromUniversal(0)),
              has_colors_(false),
              file_(std::move(file_writer)) {
      name_ = file_->GetFilename();
#ifdef WITH_OPEN3D
      pc_ = std::make_shared<open3d::geometry::PointCloud>();
      resultpc_ = std::make_shared<open3d::geometry::PointCloud>();
#endif
    }

    PointsProcessor::FlushResult MeshWritingPointsProcessor::Flush() {
#ifdef WITH_OPEN3D
      if(statistical_outlier_neighbours_ != 0 && statistical_outlier_radius_ != 0) {
        LOG(INFO) << "Removing statistical outliers using: " << std::to_string(statistical_outlier_neighbours_) << " " << std::to_string(statistical_outlier_radius_);
        std::vector<size_t> outliers;
        std::tie(resultpc_, outliers) = resultpc_->RemoveStatisticalOutliers(statistical_outlier_neighbours_, statistical_outlier_radius_);
      }

      if(poisson_depth_ == 0) {
        LOG(INFO) << "Writing point cloud to file: " + name_;
        open3d::io::WritePointCloudToPLY(name_, *resultpc_, {});
      } else {
        LOG(INFO) << "Calculating mesh using poisson reconstruction with depth: " + std::to_string(poisson_depth_);
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_es;
        std::vector<double> densities_es;
        std::tie(mesh_es, densities_es) = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*resultpc_, poisson_depth_);

        std::vector<bool> density_mask(densities_es.size(), false);
        double max = 0;
        for (int i = 0 ; i < densities_es.size(); i++) {
          if(densities_es[i] > max) max = densities_es[i];
          if(densities_es[i] < trim_surface_) density_mask[i] = true;
        }
        if(trim_surface_ > 0) {
          LOG(INFO) << "Trimming Mesh below density: " + std::to_string(trim_surface_);
          mesh_es->RemoveVerticesByMask(density_mask);
        }
        mesh_es->RemoveDegenerateTriangles();
        mesh_es->RemoveDuplicatedTriangles();
        mesh_es->RemoveDuplicatedVertices();
        mesh_es->RemoveNonManifoldEdges();

        open3d::io::WriteTriangleMesh(name_, *mesh_es);
      }

      switch (next_->Flush()) {
        case FlushResult::kFinished:
          return FlushResult::kFinished;

        case FlushResult::kRestartStream:
          LOG(FATAL) << "Mesh generation must be configured to occur after any "
                        "stages that require multiple passes.";
      }
#endif
      LOG(FATAL);
    }

    void MeshWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
      if (batch->points.empty()) {
        next_->Process(std::move(batch));
        return;
      }

      if(aggregation_counter_ == 0) {
        num_points_ = 0;
      }

#ifdef WITH_OPEN3D
      for (size_t i = 0; i < batch->points.size(); ++i) {
        pc_->points_.push_back({
                                       batch->points[i].position[0],
                                       batch->points[i].position[1],
                                       batch->points[i].position[2]});
        if(batch->colors.size() > i) {
          pc_->colors_.push_back({
                                         batch->colors[i][0],
                                         batch->colors[i][1],
                                         batch->colors[i][2]});
        } else {
          // LOG(INFO) << "More points than colors";
        }
        ++num_points_;
      }
      ++aggregation_counter_;
      if(aggregation_counter_ >= aggregate_) {
        aggregation_counter_ = 0;
        pc_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.5, 30));
        pc_->OrientNormalsTowardsCameraLocation(batch->origin.cast<double>());
        resultpc_->operator+=(*pc_);

        pc_ = std::make_shared<open3d::geometry::PointCloud>();
      }
#endif
      next_->Process(std::move(batch));
    }

  }  // namespace io
}  // namespace cartographer
