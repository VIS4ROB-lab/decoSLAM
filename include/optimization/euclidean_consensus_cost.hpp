#pragma once

#include <cmath>
#include <vector>

#include <ceres/sized_cost_function.h>
#include <hyper/variables/cartesian.hpp>
#include <hyper/variables/jacobian.hpp>

#include "global.hpp"

namespace deco {

template <int N>
class EuclideanConsensusCost : public ceres::SizedCostFunction<N, N> {
  public:
    using Value = hyper::Cartesian<Scalar, N>;
    using Jacobian = hyper::Jacobian<Value, Value>;

    EuclideanConsensusCost() = delete;
    explicit EuclideanConsensusCost(const Scalar& gamma);
    ~EuclideanConsensusCost() = default;

    auto setDualCorrection(const Value& dual_correction, int cardinality) -> void;

    auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool override;

  private:
    const Scalar gamma_;
    Scalar factor_;
    Value dual_correction_;
};

template <int N>
EuclideanConsensusCost<N>::EuclideanConsensusCost(const Scalar& gamma)
    : gamma_{gamma},
      factor_{0} {}

template <int N>
auto EuclideanConsensusCost<N>::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool {
    Eigen::Map<const Value> value{parameters[0]};
    auto residual = Eigen::Map<Value>{residuals};
    residual = dual_correction_ / factor_ + value * factor_;
    if (jacobians) {
        Eigen::Map<Jacobian> jacobian{jacobians[0]};
        jacobian.setZero();
        jacobian = factor_ * Jacobian::Identity();
    }
    return true;
}

template <int N>
auto EuclideanConsensusCost<N>::setDualCorrection(const Value& dual_correction, const int cardinality) -> void {
    dual_correction_ = dual_correction;
    factor_ = std::sqrt(cardinality * gamma_);
}

} // namespace deco
