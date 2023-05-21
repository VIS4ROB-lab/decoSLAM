//
// Created by philipp on 27.07.22.
//

#include <hyper/sensors/camera.hpp>
#include <hyper/variables/adapters.hpp>
#include <hyper/variables/jacobian.hpp>

#include "optimization/bearing_cost.hpp"

namespace deco {

namespace {

auto normalize(const Position& pos, Scalar* jacobian = nullptr) -> Bearing {
    if (!jacobian) {
        return {pos.normalized()};
    } else {
        const auto norm = pos.norm();
        const auto factor = norm * norm * norm;
        auto normalization_jacobian = Eigen::Map<hyper::Jacobian<Bearing, Position>>{jacobian};
        normalization_jacobian(0, 0) = pos.y() * pos.y() + pos.z() * pos.z();
        normalization_jacobian(1, 1) = pos.x() * pos.x() + pos.z() * pos.z();
        normalization_jacobian(2, 2) = pos.x() * pos.x() + pos.y() * pos.y();
        normalization_jacobian(0, 1) = -pos.x() * pos.y();
        normalization_jacobian(0, 2) = -pos.x() * pos.z();
        normalization_jacobian(1, 2) = -pos.y() * pos.z();
        normalization_jacobian(1, 0) = normalization_jacobian(0, 1);
        normalization_jacobian(2, 0) = normalization_jacobian(0, 2);
        normalization_jacobian(2, 1) = normalization_jacobian(1, 2);
        normalization_jacobian /= factor;
        return {pos.normalized()};
    }
}

auto metric(const Bearing& bearing_1, const Bearing& bearing_2, Scalar* left_jacobian = nullptr, Scalar* right_jacobian = nullptr) -> Scalar {
    // Compute dot-product between observed bearing and normalized landmark, and other quantities.
    const Scalar dot_product = bearing_1.dot(bearing_2);
    const auto x = (dot_product < -1.0) ? -1.0 : ((1.0 < dot_product) ? 1.0 : dot_product);

    // Evaluate jacobian if requested.
    if (left_jacobian) {
        auto jacobian = Eigen::Map<hyper::Jacobian<hyper::Cartesian<Scalar, 1>, Bearing>>{left_jacobian};
        jacobian = -bearing_2.transpose();
    }

    if (right_jacobian) {
        auto jacobian = Eigen::Map<hyper::Jacobian<hyper::Cartesian<Scalar, 1>, Bearing>>{right_jacobian};
        jacobian = -bearing_1.transpose();
    }

    // Return residual.
    return Scalar{1} - x;
}

/*auto metric(const Bearing& bearing_1, const Bearing& bearing_2, Scalar* left_jacobian = nullptr, Scalar* right_jacobian = nullptr) -> hyper::Cartesian<Scalar, 2> {
    auto value = hyper::Cartesian<Scalar, 2>();
    value.x() = std::cos(bearing_1.x()) - std::cos(bearing_2.x());
    value.y() = std::cos(bearing_1.y()) - std::cos(bearing_2.y());
    if (left_jacobian) {
        auto jacobian = Eigen::Map<hyper::Jacobian<hyper::Cartesian<Scalar, 2>, Bearing>>{left_jacobian};
        jacobian.setZero();
        jacobian(0, 0) = -std::sin(bearing_1.x());
        jacobian(1, 1) = -std::sin(bearing_1.y());
    }
    if (right_jacobian) {
        auto jacobian = Eigen::Map<hyper::Jacobian<hyper::Cartesian<Scalar, 2>, Bearing>>{right_jacobian};
        jacobian.setZero();
        jacobian(0, 0) = std::sin(bearing_2.x());
        jacobian(1, 1) = std::sin(bearing_2.y());
    }

    return value;
}*/
} // namespace

BearingCost::BearingCost(const Bearing& measured_bearing, const Scalar& sigma)
    : measured_bearing_{measured_bearing}, sigma_{sigma} {}

auto BearingCost::Evaluate(const double* const* parameters, double* residuals, double** jacobians) const -> bool {
    // Map parameters.
    auto world_position = Eigen::Map<const Position>{parameters[index::kLandmarkIdx]};
    auto se3_world_body = Eigen::Map<const SE3>{parameters[index::kBodyPoseIdx]};
    auto se3_body_camera = Eigen::Map<const SE3>{parameters[index::kSensorPoseIdx]};

    if (!jacobians) {
        const auto se3_camera_world = (se3_world_body.groupPlus(se3_body_camera)).groupInverse();
        const Position camera_position = se3_camera_world.vectorPlus(world_position);
        const Bearing predicted_bearing = hyper::Camera::ProjectToSphere(camera_position);
        residuals[0] = metric(predicted_bearing, measured_bearing_) / sigma_;
    } else {
        hyper::Jacobian<hyper::Tangent<SE3>> J_se3_wc_se3_wb, J_se3_wc_se3_bc, J_se3_cw_se3_wc;
        const auto se3_world_camera = se3_world_body.groupPlus(se3_body_camera, J_se3_wc_se3_wb.data(), J_se3_wc_se3_bc.data());
        const auto se3_camera_world = se3_world_camera.groupInverse(J_se3_cw_se3_wc.data());
        const hyper::Jacobian<hyper::Tangent<SE3>> J_se3_cw_se3_wb = J_se3_cw_se3_wc * J_se3_wc_se3_wb;
        const hyper::Jacobian<hyper::Tangent<SE3>> J_se3_cw_se3_bc = J_se3_cw_se3_wc * J_se3_wc_se3_bc;

        hyper::Jacobian<Position, hyper::Tangent<SE3>> J_pc_se3_cw;
        hyper::Jacobian<Position, Position> J_pc_pw;
        const Position camera_position = se3_camera_world.vectorPlus(world_position, J_pc_se3_cw.data(), J_pc_pw.data());
        hyper::Jacobian<Bearing, Position> J_b_pc;
        const Bearing predicted_bearing = hyper::Camera::ProjectToSphere(camera_position, J_b_pc.data());

        hyper::Jacobian<Residual, Bearing> J_r_b;
        residuals[0] = metric(predicted_bearing, measured_bearing_, J_r_b.data()) / sigma_;

        // Landmark jacobian.
        if (jacobians[index::kLandmarkIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, Position, Eigen::RowMajor>>{jacobians[index::kLandmarkIdx]};
            jacobian = J_r_b * J_b_pc * J_pc_pw / sigma_;
        }

        // Body pose jacobian.
        if (jacobians[index::kBodyPoseIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, SE3, Eigen::RowMajor>>{jacobians[index::kBodyPoseIdx]};
            jacobian = J_r_b * J_b_pc * J_pc_se3_cw * J_se3_cw_se3_wb * hyper::SE3JacobianAdapter(se3_world_body.data()) / sigma_;
        }

        // Sensor pose jacobian.
        if (jacobians[index::kSensorPoseIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, SE3, Eigen::RowMajor>>{jacobians[index::kSensorPoseIdx]};
            jacobian = J_r_b * J_b_pc * J_pc_se3_cw * J_se3_cw_se3_bc * hyper::SE3JacobianAdapter(se3_body_camera.data()) / sigma_;
        }
    }

    return true;
}
} // namespace deco
