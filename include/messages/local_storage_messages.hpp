//
// Created by philipp on 09.08.22.
//

#pragma once

#include <cereal/types/map.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/vector.hpp>
#include <opencv2/core.hpp>

#include "map/map.hpp"
#include "messages/base.hpp"

namespace deco {
enum class LocalStorageMessageType {
    OBSERVATION_STORAGE,
    EDGE_UPDATES,
    NEW_KEYFRAMES,
    NEW_KEYFRAME_DUALS,
    KEYFRAME_DUAL_UPDATES,
    KEYFRAME_DUAL_REMOVALS,
    KEYFRAME_REMOVALS,
    NEW_MAP_POINTS,
    NEW_MAP_POINT_DUALS,
    MAP_POINT_DUAL_UPDATES,
    MAP_POINT_DUAL_REMOVALS,
    MAP_POINT_REMOVALS
};

template <>
class Message<MessageType::LOCAL_STORAGE> : public MessageBase {
  public:
    explicit Message(const LocalStorageMessageType& message_type) : MessageBase{MessageType::LOCAL_STORAGE}, subtype{message_type} {};
    ~Message() override = default;

    auto getMessageCategory() -> MessageCategory override {
        switch (subtype) {
            case LocalStorageMessageType::NEW_KEYFRAME_DUALS:
            case LocalStorageMessageType::KEYFRAME_DUAL_UPDATES:
            case LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS:
            case LocalStorageMessageType::NEW_MAP_POINT_DUALS:
            case LocalStorageMessageType::MAP_POINT_DUAL_UPDATES:
            case LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS:
                return MessageCategory::OPTIMIZATION;
            default:
                return MessageCategory::MAP_TRANSFER;
        }
    }

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<MessageBase>(this), subtype);
    }

    LocalStorageMessageType subtype;
};
using LocalStorageMessageBase = Message<MessageType::LOCAL_STORAGE>;
template <LocalStorageMessageType> class LocalStorageMessage;

template <>
class LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::OBSERVATION_STORAGE} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), keys);
    }

    std::set<std::pair<ObservationKey, StorageAction>> keys;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::EDGE_UPDATES> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::EDGE_UPDATES} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), edge_updates);
    }

    std::map<std::pair<StateId, StateId>, int> edge_updates;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAMES> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::NEW_KEYFRAMES} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), keyframes, observations, edge_updates, world_frame_id);
    }

    std::vector<std::tuple<StateId, SE3, std::vector<Frame::ImageData>, std::vector<Frame::Calibration>, hyper::Tangent<SE3>, SE3, int>> keyframes;
    std::set<ObservationKey> observations;
    std::map<std::pair<StateId, StateId>, int> edge_updates;
    Id world_frame_id;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAME_DUALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::NEW_KEYFRAME_DUALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), duals);
    }

    std::vector<std::tuple<StateId, SE3>> duals;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_UPDATES> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::KEYFRAME_DUAL_UPDATES} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), dual_updates);
    }

    std::vector<std::tuple<StateId, hyper::Tangent<SE3>, SE3, int>> dual_updates;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), keyframe_ids);
    }

    std::set<StateId> keyframe_ids;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::KEYFRAME_REMOVALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::KEYFRAME_REMOVALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), keyframe_ids);
    }

    std::set<std::pair<StateId, bool>> keyframe_ids;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINTS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::NEW_MAP_POINTS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), map_points, world_frame_id);
    }

    std::vector<std::tuple<StateId, Position, cv::Mat, Position, int>> map_points;
    std::set<StateId> refused_ids;
    Id world_frame_id;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINT_DUALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::NEW_MAP_POINT_DUALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), duals);
    }

    std::vector<StateId> duals;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_UPDATES> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::MAP_POINT_DUAL_UPDATES} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), dual_updates);
    }

    std::vector<std::tuple<StateId, Position, int>> dual_updates;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), map_point_ids);
    }

    std::set<StateId> map_point_ids;
};

template <>
class LocalStorageMessage<LocalStorageMessageType::MAP_POINT_REMOVALS> : public LocalStorageMessageBase {
  public:
    LocalStorageMessage() : LocalStorageMessageBase{LocalStorageMessageType::MAP_POINT_REMOVALS} {};
    ~LocalStorageMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<LocalStorageMessageBase>(this), map_point_ids);
    }

    std::set<std::pair<StateId, bool>> map_point_ids;
};

} // namespace deco

CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::OBSERVATION_STORAGE>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::EDGE_UPDATES>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::NEW_KEYFRAMES>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::NEW_KEYFRAME_DUALS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::KEYFRAME_DUAL_UPDATES>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::KEYFRAME_REMOVALS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::NEW_MAP_POINTS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::NEW_MAP_POINT_DUALS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::MAP_POINT_DUAL_UPDATES>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS>)
CEREAL_REGISTER_TYPE(deco::LocalStorageMessage<deco::LocalStorageMessageType::MAP_POINT_REMOVALS>)
