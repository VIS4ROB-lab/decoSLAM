//
// Created by philipp on 27.07.22.
//

#pragma once
#include <ceres/sized_cost_function.h>
#include <hyper/variables/bearing.hpp>

#include "global.hpp"

namespace deco {

class BearingCost : public ceres::SizedCostFunction<1, 3, 7, 7> {
  public:
    BearingCost() = delete;
    BearingCost(const Bearing& measured_bearing, const Scalar& sigma);
    ~BearingCost() override = default;

    auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool override;

  private:
    using Residual = hyper::Cartesian<Scalar, 1>;

    enum index {
        kLandmarkIdx,
        kBodyPoseIdx,
        kSensorPoseIdx,
    };

    Bearing measured_bearing_;
    Scalar sigma_;
};
} // namespace deco
