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

#ifndef CARTOGRAPHER_MAPPING_3D_COVARIANCE_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_COVARIANCE_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace mapping_3d {

// Penalizes translations in direction with low variance
class CovarianceCostFunction {
 public:
  CovarianceCostFunction(const double scaling_factor,
                          Eigen::Matrix3f covariance)
      : scaling_factor_(scaling_factor),
        covariance_(covariance){}

  CovarianceCostFunction(const CovarianceCostFunction&) = delete;
  CovarianceCostFunction& operator=(const CovarianceCostFunction&) = delete;

  template <typename T>
  bool operator()(const T* const start_translation,
                  const T* const end_translation,
                  T* residual) const {
    const T delta_x = end_translation[0] - start_translation[0];
    const T delta_y = end_translation[1] - start_translation[1];
    const T delta_z = end_translation[2] - start_translation[2];


    const T r_x_inv = delta_x * delta_x * T(covariance_(0,0))
            + T(2.) * delta_x * delta_y * T(covariance_(0,1))
            + T(2.) * delta_x * delta_z * T(covariance_(0,2))
            + delta_y * delta_y * T(covariance_(1,1))
            + T(2.) * delta_y * delta_z * T(covariance_(1,2))
            + delta_z * delta_z * T(covariance_(2,2));
    const T r_x = T(1) / (r_x_inv + T(1e-2));
    residual[0] =
        scaling_factor_ * (r_x);
    //LOG(INFO)<<"r "<<residual[0];
    return true;
  }

 private:
  const double scaling_factor_;
  Eigen::Matrix3f covariance_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_COVARIANCE_COST_FUNCTION_H_
