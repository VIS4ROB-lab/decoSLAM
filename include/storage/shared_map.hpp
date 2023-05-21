//
// Created by philipp on 05.10.22.
//

#pragma once

#include <boost/functional/hash.hpp>
#include <map>

#include "global.hpp"
#include "map/graph.hpp"
#include "map/map.hpp"
#include "messages/shared_map_messages.hpp"
#include "utils/processing.hpp"

namespace YAML {
class Node;
}

namespace deco {
class Communicator;
class Alignment;
class LocalStorage;
struct Frame;
struct MapPoint;
class Map;

class SharedMap {
  public:
    SharedMap(const Id& id, StorageMutexHandler& mutex_handler);
    ~SharedMap();

    auto readParameters(const YAML::Node& node) -> void;
    auto writeLogs(const FilePath& output_path) -> void;
    auto setCommunicator(Communicator* communicator) -> void;
    auto setAlignment(Alignment* alignment) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;
    auto submitMessage(std::unique_ptr<MessageBase>&& message) -> void;

    auto applyTransformation(const Id& new_reference_frame_id, const SE3& transformation) -> void;

    auto storeKeyframe(std::unique_ptr<Frame>&& frame) -> Frame*;
    auto getKeyframe(const StateId& frame_id) -> Frame*;
    auto storeMapPoints(std::vector<std::unique_ptr<MapPoint>>& map_points) -> void;
    auto observationStorage(const std::set<ObservationKey>& keys, const StorageAction& storage_action) -> void;

    auto getMap() const -> const Map*;
    auto isCovisible(const StateId& frame_id_A, const StateId& frame_id_B) -> bool;
    auto mergeMapPoints(const std::vector<std::pair<StateId, StateId>>& mergeable_pairs) -> void;

  private:
    using ObservationStore = std::pair<ObservationKey, StorageAction>;

    auto observationStorage_(SharedMapMessageBase& message) -> void;
    auto preprocessObservations_(std::vector<ObservationStore>& keys) -> std::pair<std::vector<ObservationStore>, std::vector<ObservationStore>>;
    auto processMapPointObservation_(const ObservationKey& key, const StorageAction& storage_action) -> std::pair<std::set<std::pair<StateId, StateId>>, int>;
    auto updateEdges_(SharedMapMessageBase& message) -> void;

    auto handleMergeProposals_(SharedMapMessageBase& message) -> void;
    auto handleMergeRequests_(SharedMapMessageBase& message) -> void;
    auto handleMergeConfirmations_(SharedMapMessageBase& message) -> void;

    auto handleKeyframeRequests_(SharedMapMessageBase& message) -> void;
    auto handleKeyframeSynchronizations_(SharedMapMessageBase& message) -> void;
    auto sendKeyframes_(const Id& new_owner_id, const std::set<std::pair<StateId, bool>>& keyframe_requests) -> void;
    auto handleMapPointRequests_(SharedMapMessageBase& message) -> void;
    auto handleMapPointSynchronizations_(SharedMapMessageBase& message) -> void;
    auto sendMapPoints_(const Id& new_owner_id, const std::set<std::pair<StateId, bool>>& map_point_requests, const std::set<StateId>& refused_ids) -> void;
    auto updateKeyframes_(SharedMapMessageBase& message) -> void;
    auto updateMapPoints_(SharedMapMessageBase& message) -> void;
    auto tryRemoveKeyframeOwner_(SharedMapMessageBase& message) -> void;
    auto tryRemoveMapPointOwner_(SharedMapMessageBase& message) -> void;

    Id id_;

    Scalar admm_gamma_;
    Scalar admm_stepsize_;
    Scalar dual_parameter_tolerance_;

    Communicator* communicator_;
    LocalStorage* local_storage_;
    Alignment* alignment_;

    enum class Status {
        ACTIVE,
        MERGING,
        MERGED,
        REMOVED
    };

    template <typename StateType>
    struct StorageEntry {
        using DualId = std::pair<Id, Id>;

        Status status;                                          ///< The status of the state.
        Id reference_frame_id;                                  ///< The id of the reference frame in which this state is expressed.
        std::set<Id> owners;                                    ///< Ids of agents having this state in their local storage.
        std::map<Id, bool> requesters;                          ///< Ids of agents having requested this state and whether constant data is needed.
        std::map<Id, StateType> estimates;                      ///< Per agent estimates of the state.
        StateType reference_state;                              ///< Initial value of the state at creation.
        std::map<DualId, hyper::Tangent<StateType>> dual_table; ///< Table of duals for this state.
    };

    std::unordered_map<StateId, std::pair<std::unique_ptr<Frame>, StorageEntry<SE3>>, Hash<StateId>> keyframes_;
    std::unordered_map<StateId, std::pair<std::unique_ptr<MapPoint>, StorageEntry<Position>>, Hash<StateId>> map_points_;
    std::unordered_map<StateId, StateId, Hash<StateId>> map_point_merges_;
    std::deque<std::set<StateId>> new_map_point_queue_;

    Id reference_frame_id_;
    Map map_;
    Graph<StateId> covisibility_graph_;

    StorageMutexHandler& mutex_handler_;
};

} // namespace deco
