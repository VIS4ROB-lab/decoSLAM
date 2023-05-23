//
// Created by philipp on 28.07.22.
//

#pragma once
#include <ceres/local_parameterization.h>

namespace deco {

class PoseParametrization : public ceres::ProductParameterization {
  public:
    PoseParametrization()
        : ceres::ProductParameterization{new ceres::EigenQuaternionParameterization{}, new ceres::IdentityParameterization{3}} {}
    ~PoseParametrization() override = default;
};
} // namespace deco
