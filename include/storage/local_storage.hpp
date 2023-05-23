//
// Created by philipp on 05.10.22.
//

#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "global.hpp"
#include "map/frame.hpp"
#include "map/graph.hpp"
#include "map/map.hpp"
#include "messages/local_storage_messages.hpp"
#include "utils/processing.hpp"

namespace YAML {
class Node;
}

namespace deco {
class MessageBase;
class Communicator;
class SharedMap;
class LocalTracking;
class Alignment;
class Visualizer;

class LocalStorage {
  public:
    LocalStorage(const Id& id, StorageMutexHandler& mutex_handler);
    LocalStorage() = delete;
    ~LocalStorage();

    auto readParameters(const YAML::Node& node) -> void;
    auto setCommunicator(Communicator* communicator) -> void;
    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalTracking(LocalTracking* local_tracking) -> void;
    auto setAlignment(Alignment* alignment) -> void;
    auto setVisualizer(Visualizer* visualizer) -> void;

    auto start() -> void;
    auto stop() -> void;
    auto writePoses(const FilePath& output_path) -> void;
    auto writeLogs(const FilePath& output_path) -> void;
    auto submitMessage(std::unique_ptr<MessageBase>&& message) -> void;

    auto setTrackingReference(const StateId& frame_id) -> void;
    auto notifyTracker() -> void;
    auto notifyOptimization() -> void;
    auto addMapPointsToCullingQueue(const std::set<StateId>& map_point_ids) -> void;
    auto registerKeyframeForDriftCorrection(const StateId& frame_id) -> void;
    auto registerMapPointsForDriftCorrection(const std::set<StateId>& map_point_ids) -> void;
    auto applyDriftCorrection(const SE3& drift) -> void;
    auto requestKeyframeRemovals() -> void;
    auto applyTransformation(const Id& new_world_frame_id, const SE3& transformation) -> void;

    auto addConstantKeyframeData(const StateId& frame_id, const std::vector<Frame::ImageData>& image_data, const std::vector<Frame::Calibration>& calibration) -> void;
    auto getKeyframeImageData(const StateId& frame_id) -> std::vector<Frame::ImageData>*;
    auto getKeyframeCalibration(const StateId& frame_id) -> std::vector<Frame::Calibration>;
    auto addKeyframePose(const StateId& frame_id, const SE3& pose) -> void;
    auto hasKeyframePose(const StateId& frame_id) -> bool;
    auto getKeyframePose(const StateId& frame_id) -> SE3;

    auto addConstantMapPointData(const StateId& map_point_id, const cv::Mat& descriptor) -> void;
    auto getMapPointDescriptor(const StateId& map_point_id) -> cv::Mat;
    auto addMapPointPosition(const StateId& map_point_id, const Position& position) -> void;
    auto hasMapPointPosition(const StateId& map_point_id) -> bool;
    auto getMapPointPosition(const StateId& map_point_id) -> Position;

    auto getMap() -> Map*;

    auto triggerVisualization() -> void;

  private:
    struct KeyframeEntry;
    struct MapPointEntry;

    auto fetchOrCreateKeyframeEntry_(const StateId& frame_id) -> KeyframeEntry&;
    auto fetchOrCreateMapPointEntry_(const StateId& map_point_id) -> MapPointEntry&;

    auto observationStorage_(LocalStorageMessageBase& message) -> void;
    auto updateEdges_(LocalStorageMessageBase& message) -> void;
    auto addNewKeyframes_(LocalStorageMessageBase& message) -> void;
    auto addNewKeyframeDuals_(LocalStorageMessageBase& message) -> void;
    auto updateKeyframeDuals_(LocalStorageMessageBase& message) -> void;
    auto removeKeyframeDuals_(LocalStorageMessageBase& message) -> void;
    auto removeKeyframes_(LocalStorageMessageBase& message) -> void;
    auto addNewMapPoints_(LocalStorageMessageBase& message) -> void;
    auto addNewMapPointDuals_(LocalStorageMessageBase& message) -> void;
    auto updateMapPointDuals_(LocalStorageMessageBase& message) -> void;
    auto removeMapPointDuals_(LocalStorageMessageBase& message) -> void;
    auto removeMapPoints_(LocalStorageMessageBase& message) -> void;

    auto loadLocalNeighborhood_(const StateId& tracking_reference_id) -> void;

    auto trackingUpdate_() -> void;
    auto optimize_() -> void;

    Id id_;
    bool do_map_sharing_;

    SharedMap* shared_map_;
    Communicator* communicator_;
    LocalTracking* local_tracking_;
    Alignment* alignment_;

    enum class StorageStatus {
        EMPTY,
        REQUESTED,
        ACTIVE,
        OPTIMIZED,
        REMOVAL_REQUESTED,
        REMOVED
    };

    struct KeyframeEntry {
        StorageStatus storage_status = StorageStatus::EMPTY; ///< Storage status indicator.
        bool is_local = false;                               ///< Whether the pose is within the second-order neighborhood of the current reference keyframe.

        // Constant data.
        std::vector<Frame::ImageData> image_data;    ///< Image data.
        std::vector<Frame::Calibration> calibration; ///< Calibration
        bool has_constant_data = false;              ///< Whether image data and calibration have been assigned.

        // Estimation.
        SE3 pose;                 ///< Current pose estimate.
        SE3 optimizable_pose;     ///< Copy of pose to be used by optimization.
        SE3 reference_pose;       ///< Reference pose used for consensus.
        hyper::Tangent<SE3> dual; ///< Dual factor.
        int cardinality = 0;      ///< Cardinality of the dual factor.
        bool has_dual = false;    ///< Whether the pose is shared.
    };

    std::map<StateId, KeyframeEntry> keyframes_;
    std::set<StateId> local_keyframe_ids_;
    std::set<StateId> new_keyframe_ids_;

    struct MapPointEntry {
        StorageStatus storage_status = StorageStatus::EMPTY; ///< Storage status indicator.
        bool is_local = false;                               ///< Whether the map point is within the second-order neighborhood of the current reference keyframe.

        // Constant data.
        cv::Mat descriptor;             ///< Map point descriptor.
        bool has_constant_data = false; ///< Whether the descriptor has been assigned.

        // Estimation.
        Position position;             ///< Current position estimate.
        Position optimizable_position; ///< Copy of position to be used for optimization.
        hyper::Tangent<Position> dual; ///< Dual factor.
        int cardinality = 0;           ///< Cardinality of the dual factor.
        bool has_dual = false;         ///< Whether the position is shared.
    };

    std::map<StateId, MapPointEntry> map_points_;
    std::set<StateId> local_map_point_ids_;
    std::deque<StateId> map_point_culling_queue_;
    std::set<StateId> new_map_point_ids_;

    Map map_;
    Graph<StateId> covisibility_graph_;
    Id world_frame_id_;

    SE3 drift_;
    StateId tracking_reference_;
    std::thread tracking_update_thread_;                        ///< Thread.
    mutable std::mutex tracking_update_mutex_;                  ///< Mutex.
    mutable std::condition_variable tracking_update_condition_; ///< Queue condition.
    mutable bool tracking_update_notified_;                     ///< Notified flag.

    std::thread optimization_thread_;
    mutable std::mutex optimization_mutex_;
    mutable std::condition_variable optimization_condition_;
    mutable bool optimization_notified_;

    // Optimization Parameters.
    Scalar pixel_noise_;
    Scalar huber_loss_threshold_;
    Scalar outlier_threshold_;
    Scalar admm_gamma_;
    int max_num_iterations_;
    int optimizer_rate_us_;

    StorageMutexHandler& mutex_handler_;

    // Logging.
    std::map<Timestamp, std::pair<size_t, size_t>> map_size_log_;

    Visualizer* visualizer_;
};

} // namespace deco
