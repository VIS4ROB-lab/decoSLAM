//
// Created by philipp on 28.07.22.
//

#include <hyper/sensors/camera.hpp>
#include <hyper/variables/adapters.hpp>
#include <hyper/variables/distortions/radial_tangential.hpp>
#include <hyper/variables/intrinsics.hpp>
#include <hyper/variables/jacobian.hpp>

#include "optimization/pixel_cost.hpp"

namespace deco {

PixelCost::PixelCost(const Pixel& measured_pixel, const Scalar& sigma)
    : measured_pixel_{measured_pixel}, sigma_{sigma} {}

auto PixelCost::Evaluate(const double* const* parameters, double* residuals, double** jacobians) const -> bool {
    // Map parameters.
    auto landmark_world = Eigen::Map<const Position>{parameters[index::kLandmarkIdx]};
    auto se3_world_body = Eigen::Map<const SE3>{parameters[index::kBodyPoseIdx]};
    auto se3_body_camera = Eigen::Map<const SE3>{parameters[index::kSensorPoseIdx]};
    auto intrinsics = Eigen::Map<const hyper::Intrinsics<Scalar>>{parameters[kIntrinsicsIdx]};
    auto distortion = Eigen::Map<const hyper::RadialTangentialDistortion<Scalar, 2>>{parameters[kDistortionIdx]};

    if (!jacobians) {
        const auto se3_world_camera = se3_world_body.groupPlus(se3_body_camera);
        const auto se3_camera_world = se3_world_camera.groupInverse();
        const Position landmark_camera = se3_camera_world.vectorPlus(landmark_world);
        const auto pixel_normalized = hyper::Camera::ProjectToPlane(landmark_camera);
        const auto pixel_distorted = distortion.distort(pixel_normalized, nullptr, nullptr);
        const auto pixel_denormalized = intrinsics.denormalize(pixel_distorted, nullptr, nullptr);
        Eigen::Map<Residual>{residuals} = (pixel_denormalized - measured_pixel_) / sigma_;
    } else {
        hyper::Jacobian<hyper::Tangent<SE3>> J_se3_wc_se3_wb, J_se3_wc_se3_bc, J_se3_cw_se3_wc;
        const auto se3_world_camera = se3_world_body.groupPlus(se3_body_camera, J_se3_wc_se3_wb.data(), J_se3_wc_se3_bc.data());
        const auto se3_camera_world = se3_world_camera.groupInverse(J_se3_cw_se3_wc.data());
        const hyper::Jacobian<hyper::Tangent<SE3>> J_se3_cw_se3_wb = J_se3_cw_se3_wc * J_se3_wc_se3_wb;
        const hyper::Jacobian<hyper::Tangent<SE3>> J_se3_cw_se3_bc = J_se3_cw_se3_wc * J_se3_wc_se3_bc;

        hyper::Jacobian<Position, hyper::Tangent<SE3>> J_lc_se3_cw;
        hyper::Jacobian<Position, Position> J_lc_lw;
        const Position landmark_camera = se3_camera_world.vectorPlus(landmark_world, J_lc_se3_cw.data(), J_lc_lw.data());

        hyper::Jacobian<Pixel, Position> J_pn_lc;
        const auto pixel_normalized = hyper::Camera::ProjectToPlane(landmark_camera, J_pn_lc.data());
        hyper::Jacobian<Pixel, Pixel> J_pd_pn;
        const auto pixel_distorted = distortion.distort(pixel_normalized, J_pd_pn.data(), nullptr);
        hyper::Jacobian<Pixel, Pixel> J_pdn_pd;
        const auto pixel_denormalized = intrinsics.denormalize(pixel_distorted, J_pdn_pd.data(), nullptr);
        Eigen::Map<Residual>{residuals} = (pixel_denormalized - measured_pixel_) / sigma_;

        // Landmark jacobian.
        if (jacobians[index::kLandmarkIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, Position, Eigen::RowMajor>>{jacobians[index::kLandmarkIdx]};
            jacobian = J_pdn_pd * J_pd_pn * J_pn_lc * J_lc_lw / sigma_;
        }

        // Body pose jacobian.
        if (jacobians[index::kBodyPoseIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, SE3, Eigen::RowMajor>>{jacobians[index::kBodyPoseIdx]};
            jacobian = J_pdn_pd * J_pd_pn * J_pn_lc * J_lc_se3_cw * J_se3_cw_se3_wb * hyper::SE3JacobianAdapter(se3_world_body.data()) / sigma_;
        }

        // Sensor pose jacobian.
        if (jacobians[index::kSensorPoseIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, SE3, Eigen::RowMajor>>{jacobians[index::kSensorPoseIdx]};
            jacobian = J_pdn_pd * J_pd_pn * J_pn_lc * J_lc_se3_cw * J_se3_cw_se3_bc * hyper::SE3JacobianAdapter(se3_body_camera.data()) / sigma_;
        }

        if (jacobians[index::kIntrinsicsIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, hyper::Intrinsics<Scalar>, Eigen::RowMajor>>{jacobians[index::kIntrinsicsIdx]};
            hyper::Jacobian<Pixel, hyper::Intrinsics<Scalar>> J_pdn_in;
            intrinsics.denormalize(pixel_distorted, nullptr, J_pdn_in.data());
            jacobian = J_pdn_in / sigma_;
        }

        if (jacobians[index::kDistortionIdx]) {
            auto jacobian = Eigen::Map<hyper::Jacobian<Residual, hyper::RadialTangentialDistortion<Scalar, 2>, Eigen::RowMajor>>{jacobians[index::kDistortionIdx]};
            hyper::Jacobian<Pixel, hyper::RadialTangentialDistortion<Scalar, 2>> J_pd_dist;
            distortion.distort(pixel_normalized, nullptr, J_pd_dist.data());
            jacobian = J_pdn_pd * J_pd_dist / sigma_;
        }
    }

    return true;
}
} // namespace deco
