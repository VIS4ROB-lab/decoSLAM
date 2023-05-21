//
// Created by philipp on 21.06.22.
//

#pragma once

#include <future>

#include <hyper/sensors/camera.hpp>

#include "map/frame.hpp"

namespace deco {

/// Extract bearings and descriptors from a set of images.
/// \tparam Detector Keypoint detector type.
/// \param images Vector of input images.
/// \param cameras Cameras corresponding to images.
/// \param detector Keypoint detector.
/// \return Vector of features (keypoints and bearings).
template <class Detector>
auto extractImageData(
    const std::vector<const cv::Mat*>& images,
    const std::vector<hyper::Camera*>& cameras,
    const Detector& detector) -> std::vector<Frame::ImageData> {
    DCHECK(images.size() == cameras.size());

    std::vector<Frame::ImageData> camera_features;
    camera_features.reserve(images.size());

    auto task_lambda = [&](const cv::Mat& image, const hyper::Camera* camera) -> Frame::ImageData {
        Frame::ImageData image_data;

        // Apply histogram equalization
        // cv::equalizeHist(image, equalized_img);

        // Detect keypoints.
        detector->detect(image, image_data.keypoints);

        // Compute descriptors.
        detector->compute(image, image_data.keypoints, image_data.keypoint_descriptors);

        std::vector<Pixel> pixels;
        for (const auto& keypoint : image_data.keypoints) {
            pixels.emplace_back(keypoint.pt.x, keypoint.pt.y);
        }
        image_data.bearings = camera->convertPixelsToBearings(pixels);
        return image_data;
    };

    auto futures = std::vector<std::future<Frame::ImageData>>();
    for (auto i = 0u; i < images.size(); ++i) {
        futures.emplace_back(std::async(std::launch::async, std::bind(task_lambda, *images[i], cameras[i])));
    }

    for (auto& future : futures) {
        future.wait();
        camera_features.emplace_back(future.get());
    }

    return camera_features;
}

} // namespace deco
