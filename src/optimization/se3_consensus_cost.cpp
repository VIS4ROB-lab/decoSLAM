//
// Created by philipp on 03.08.22.
//

#include <hyper/variables/adapters.hpp>

#include "optimization/se3_consensus_cost.hpp"

namespace deco {

SE3ConsensusCost::SE3ConsensusCost(const Scalar& gamma, const SE3& se3_reference)
    : gamma_{gamma},
      factor_{0},
      se3_reference_{se3_reference} {}

auto SE3ConsensusCost::setDualCorrection(const Dual& dual_correction, const int cardinality) -> void {
    dual_correction_ = dual_correction;
    factor_ = std::sqrt(cardinality * gamma_);
}

auto SE3ConsensusCost::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const -> bool {
    SE3 se3 = Eigen::Map<const SE3>{parameters[0]};
    const Dual value = (se3_reference_.groupInverse().groupPlus(se3)).toTangent();
    auto residual = Eigen::Map<Dual>{residuals};
    residual = dual_correction_ / factor_ + value * factor_;
    if (jacobians) {
        hyper::Jacobian<hyper::Tangent<SE3>> J_diff_se3;
        const SE3 se3_diff = se3_reference_.groupInverse().groupPlus(se3, nullptr, J_diff_se3.data());
        hyper::Jacobian<hyper::Tangent<SE3>> J_value_se3diff;
        se3_diff.toTangent(J_value_se3diff.data());
        hyper::Jacobian<Dual, hyper::Tangent<SE3>> J_r_value;
        J_r_value.setIdentity();
        J_r_value *= factor_;

        auto jacobian = Eigen::Map<hyper::Jacobian<Dual, SE3, Eigen::RowMajor>>{jacobians[0]};
        jacobian = J_r_value * J_value_se3diff * J_diff_se3 * hyper::SE3JacobianAdapter(se3.data());
    }
    return true;
}

} // namespace deco
