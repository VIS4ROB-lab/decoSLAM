//
// Created by philipp on 28.07.22.
//

#pragma once
#include <ceres/sized_cost_function.h>
#include <hyper/variables/cartesian.hpp>

#include "global.hpp"

namespace deco {

class PixelCost : public ceres::SizedCostFunction<2, 3, 7, 7, 4, 4> {
  public:
    PixelCost() = delete;
    PixelCost(const Pixel& measured_pixel, const Scalar& sigma);
    ~PixelCost() override = default;

    auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool override;

  private:
    using Residual = hyper::Cartesian<Scalar, 2>;

    enum index {
        kLandmarkIdx,
        kBodyPoseIdx,
        kSensorPoseIdx,
        kIntrinsicsIdx,
        kDistortionIdx,
    };

    Pixel measured_pixel_;
    Scalar sigma_;
};
} // namespace deco
