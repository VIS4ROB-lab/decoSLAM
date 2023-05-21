//
// Created by philipp on 04.10.22.
//

#pragma once

#include <mutex>
#include <vector>

#include <cereal/types/vector.hpp>
#include <hyper/variables/bearing.hpp>
#include <hyper/variables/distortions/radial_tangential.hpp>
#include <hyper/variables/groups/se3.hpp>
#include <hyper/variables/intrinsics.hpp>
#include <opencv2/core/core.hpp>

#include "global.hpp"

namespace deco {

struct Frame {
    struct ImageData {
        std::vector<cv::KeyPoint> keypoints; ///< Keypoints from an image.
        std::vector<Bearing> bearings;       ///< Bearings for the keypoints.
        cv::Mat keypoint_descriptors;        ///< Descriptors for the keypoints.
        cv::Mat image_descriptor;            ///< Full image descriptor.

        template <class Archive>
        auto serialize(Archive& archive) -> void {
            archive(keypoints, bearings, keypoint_descriptors, image_descriptor);
        }
    };

    struct Calibration {
        SE3 se3_body_camera_; ///< Transformations of the cameras with respect to the body frame.
        hyper::Intrinsics<Scalar> intrinsics_;
        hyper::RadialTangentialDistortion<Scalar, 2> distortions_;

        template <class Archive>
        auto serialize(Archive& archive) -> void {
            archive(se3_body_camera_, intrinsics_, distortions_);
        }
    };

    StateId id;                                   ///< Unique Id of the frame.
    Timestamp timestamp;                          ///< Timestamp of the frame.
    std::vector<cv::Mat> images;                  ///< Raw images.
    std::vector<ImageData> image_data;            ///< Per camera image data.
    std::vector<Calibration> calibration;         ///< Per camera extrinsic + intrinsic calibration.
    Eigen::Matrix<Scalar, 3, 3> essential_matrix; ///< Essential matrix (left-to-right).
    SE3 se3_world_body;                           ///< Transformation of the frame with respect to the world frame.
    Id reference_frame_id;

    std::mutex mutex_;
};

} // namespace deco
