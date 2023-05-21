//
// Created by philipp on 03.10.22.
//

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <hyper/variables/groups/se3.hpp>
#include <opencv2/features2d.hpp>

#include "global.hpp"
#include "map/frame.hpp"

namespace cv {
class Mat;
}

namespace YAML {
class Node;
}

namespace hyper {
class Camera;
}

namespace deco {
class SharedMap;
class LocalStorage;
class LocalMapping;
class Alignment;
class Visualizer;

class LocalTracking {
  public:
    using Images = std::vector<const cv::Mat*>;

    struct TrackingMap {
        StateId reference_keyframe_id;                                 ///< Id of the reference keyframe.
        SE3 reference_keyframe_pose;                                   ///< Pose of the reference keyframe.
        std::map<StateId, std::set<StateId>> local_slam_graph;         ///< Local SLAM graph.
        std::map<StateId, std::pair<Translation, cv::Mat>> map_points; ///< Map point ids in the local SLAM graph.
        // std::vector<Translation> map_point_positions;          ///< Map point positions in the local SLAM graph.
        // cv::Mat map_point_descriptors;                         ///< Map point descriptors in the local SLAM graph.
        Id world_frame_id;
    };

    LocalTracking() = delete;
    LocalTracking(const Id& id, StorageMutexHandler& mutex_handler);
    ~LocalTracking() = default;

    auto readParameters(const YAML::Node& node) -> void;
    auto setMatchingParameters(const MatchingParameters& parameters) -> void;
    auto setCameras(const std::vector<hyper::Camera*>& cameras) -> void;
    auto setLocalMapping(LocalMapping* local_mapping) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;
    auto setAlignment(Alignment* alignment) -> void;
    auto setVisualizer(Visualizer* visualizer) -> void;

    auto mutex() -> std::mutex&;
    auto extendTrackingMap(const StateId& frame_id, const std::vector<std::tuple<StateId, Translation, cv::Mat>>& new_map_points) -> void;
    auto setTrackingMap(std::unique_ptr<TrackingMap>&& map) -> void;
    auto applyDriftCorrection(const SE3& drift) -> void;
    auto callback(boost::shared_ptr<const void> message, size_t cam_idx) -> void;
    auto submit(const Images& images, const Timestamp& timestamp) -> void;

  private:
    using ImagePtr = cv_bridge::CvImageConstPtr;
    using ImageMap = std::map<size_t, ImagePtr>;

    auto initialize_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> void;
    auto localize_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> void;

    auto generateNewKeyframe_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> std::unique_ptr<Frame>;

    // Id
    Id id_; ///< Id of the agent owning this frontend.

    // State.
    SE3 se3_world_body_current_; ///< Transformation of the current frame w.r.t. the world frame.
    Id world_frame_id_;

    // Feature extractor.
    cv::Ptr<cv::BRISK> detector_; ///< BRISK feature extractor object.

    // Cameras.
    std::vector<hyper::Camera*> cameras_;                     ///< Pointers to cameras used by this frontend.
    SE3 se3_left_right_;                                      ///< Transformation of the right camera w.r.t. the left camera.
    Eigen::Matrix<Scalar, 3, 3> essential_matrix_left_right_; ///< Essential matrix.

    // Map portion used for tracking.
    std::mutex map_mutex_;
    std::unique_ptr<TrackingMap> map_;

    // Keyframe creation parameters.
    Id keyframe_count_;             ///< Number of keyframes created so far (used for keyframe identification).
    size_t min_num_points_tracked_; ///< Minimum number of tracked points.
    Scalar baseline_depth_ratio_;   ///< Ratio between baseline and average depth of observed map points.
    bool is_map_reset_;

    // Matching parameters.
    MatchingParameters parameters_;

    // Storage
    StorageMutexHandler& mutex_handler_;
    LocalStorage* local_storage_; ///< Local Storage.

    // Connected modules.
    LocalMapping* local_mapping_; ///< Local Mapping.
    Alignment* alignment_;

    // Queues.
    std::map<Timestamp, ImageMap> image_queue_; ///< Image queue accumulating stereo frames.

    cv::Scalar color_ = {255, 0, 0};
    Visualizer* visualizer_;
};
} // namespace deco
