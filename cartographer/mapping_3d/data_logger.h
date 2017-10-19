#ifndef CARTOGRAPHER_MAPPING_3D_DATA_LOGGER_H_
#define CARTOGRAPHER_MAPPING_3D_DATA_LOGGER_H_

#include <array>
#include <memory>

#include "Eigen/Core"
#include "cartographer/transform/rigid_transform.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping_3d {

class CeresPose {
 public:
  CeresPose(
      const transform::Rigid3d& rigid,
      std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
      std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
      ceres::Problem* problem);

  CeresPose(const CeresPose&) = delete;
  CeresPose& operator=(const CeresPose&) = delete;

  const transform::Rigid3d ToRigid() const;

  double* translation() { return translation_.data(); }
  double* rotation() { return rotation_.data(); }

 private:
  std::array<double, 3> translation_;
  // Rotation quaternion as (w, x, y, z).
  std::array<double, 4> rotation_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_CERES_POSE_H_
