//
// Created by philipp on 03.08.22.
//

#pragma once

#include <ceres/sized_cost_function.h>
#include <hyper/variables/cartesian.hpp>
#include <hyper/variables/jacobian.hpp>

#include "global.hpp"

namespace deco {
constexpr auto kNumSE3TangentParameters = hyper::Traits<hyper::Tangent<SE3>>::kNumParameters;

class SE3ConsensusCost : public ceres::SizedCostFunction<kNumSE3TangentParameters, hyper::Traits<SE3>::kNumParameters> {
  public:
    using Dual = hyper::Tangent<SE3>;

    SE3ConsensusCost() = delete;
    explicit SE3ConsensusCost(const Scalar& gamma, const SE3& se3_reference);
    ~SE3ConsensusCost() override = default;

    auto setDualCorrection(const Dual& dual_correction, int cardinality) -> void;

    auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool override;

  private:
    const Scalar gamma_;
    Scalar factor_;
    Dual dual_correction_;
    SE3 se3_reference_;
};

} // namespace deco
