//
// Created by philipp on 08.11.22.
//

#pragma once
/*
#include <condition_variable>
#include <mutex>
#include <thread>

#include "global.hpp"

namespace YAML {
class Node;
}

namespace deco {
class SharedMap;
class LocalStorage;
class Communicator;
class Alignment;

class Optimizer {
  public:
    explicit Optimizer(StorageMutexHandler& mutex_handler);
    ~Optimizer();

    auto readParameters(const YAML::Node& node) -> void;
    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;
    auto setCommunicator(Communicator* communicator) -> void;
    auto setAlignment(Alignment* alignment) -> void;

    auto start() -> void;
    auto stop() -> void;
    auto notify() -> void;

    auto setDriftReference(const StateId& drift_reference_id) -> void;

    auto writeLogs(const FilePath& output_path) -> void;

  private:
    auto spin_() -> void;

    // Connected Modules
    StorageMutexHandler& mutex_handler_;
    SharedMap* shared_map_;
    LocalStorage* local_storage_;
    Alignment* alignment_;
    Communicator* communicator_;

    // Internal copies.
    std::unordered_map<StateId, SE3, Hash<StateId>> keyframe_poses_;
    std::unordered_map<StateId, std::vector<Frame::Calibration>, Hash<StateId>> keyframe_calibrations_;
    std::unordered_map<StateId, std::tuple<hyper::Tangent<SE3>, SE3, int>, Hash<StateId>> keyframe_duals_;
    std::unordered_map<StateId, Position, Hash<StateId>> map_point_positions_;
    std::unordered_map<StateId, std::pair<Position, int>, Hash<StateId>> map_point_duals_;

    // Parameters.
    Scalar pixel_noise_;
    Scalar huber_loss_threshold_;
    Scalar outlier_threshold_;
    Scalar admm_gamma_;
    int max_num_iterations_;

    StateId drift_reference_id_;
    std::thread optimization_thread_;              ///< Thread.
    mutable std::mutex optimization_thread_mutex_; ///< Mutex.
    mutable std::condition_variable condition_;    ///< Queue condition.
    mutable bool notified_;                        ///< Notified flag.

    // Logging.
    std::map<Timestamp, std::pair<size_t, size_t>> map_size_log_;
};
} // namespace deco
 */
