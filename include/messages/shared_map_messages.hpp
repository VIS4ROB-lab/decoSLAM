//
// Created by philipp on 09.08.22.
//

#pragma once

#include <cereal/types/map.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/set.hpp>

#include "map/frame.hpp"
#include "messages/base.hpp"

namespace deco {
enum class SharedMapMessageType {
    OBSERVATION_STORAGE,
    EDGE_UPDATES,
    MAP_POINT_MERGE_PROPOSALS,
    MAP_POINT_MERGE_REQUESTS,
    MAP_POINT_MERGE_CONFIRMATIONS,
    KEYFRAME_REQUESTS,
    KEYFRAME_SYNCHRONIZATIONS,
    KEYFRAME_UPDATES,
    KEYFRAME_DELETION_REQUESTS,
    MAP_POINT_REQUESTS,
    MAP_POINT_SYNCHRONIZATIONS,
    MAP_POINT_UPDATES,
    MAP_POINT_REMOVAL_REQUESTS
};

template <>
class Message<MessageType::SHARED_MAP> : public MessageBase {
  public:
    explicit Message(const SharedMapMessageType& message_type) : MessageBase{MessageType::SHARED_MAP}, subtype{message_type} {};
    ~Message() override = default;

    auto getMessageCategory() -> MessageCategory override {
        switch (subtype) {
            case SharedMapMessageType::KEYFRAME_UPDATES:
            case SharedMapMessageType::MAP_POINT_UPDATES:
                return MessageCategory::OPTIMIZATION;
            default:
                return MessageCategory::MAP_TRANSFER;
        }
    }

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<MessageBase>(this), subtype);
    }

    SharedMapMessageType subtype;
};
using SharedMapMessageBase = Message<MessageType::SHARED_MAP>;
template <SharedMapMessageType> class SharedMapMessage;

template <>
class SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::OBSERVATION_STORAGE} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), keys);
    }

    std::vector<std::pair<ObservationKey, StorageAction>> keys;
};

template <>
class SharedMapMessage<SharedMapMessageType::EDGE_UPDATES> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::EDGE_UPDATES} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), edge_updates);
    }

    std::map<std::pair<StateId, StateId>, int> edge_updates;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), mergeable_pairs);
    }

    std::set<std::pair<StateId, StateId>> mergeable_pairs;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_REQUESTS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_MERGE_REQUESTS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), mergeable_pairs);
    }

    std::set<std::pair<StateId, StateId>> mergeable_pairs;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), map_point_ids);
    }

    std::set<StateId> map_point_ids;
};

template <>
class SharedMapMessage<SharedMapMessageType::KEYFRAME_REQUESTS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::KEYFRAME_REQUESTS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), keyframe_ids);
    }

    std::set<std::pair<StateId, bool>> keyframe_ids;
};

template <>
class SharedMapMessage<SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), keyframe_poses_);
    }

    std::map<StateId, SE3> keyframe_poses_;
};

template <>
class SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::KEYFRAME_UPDATES} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), keyframe_updates);
    }

    std::vector<std::pair<StateId, SE3>> keyframe_updates;
};

template <>
class SharedMapMessage<SharedMapMessageType::KEYFRAME_DELETION_REQUESTS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::KEYFRAME_DELETION_REQUESTS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), keyframe_ids);
    }

    std::set<StateId> keyframe_ids;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_REQUESTS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_REQUESTS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), map_point_ids);
    }

    std::set<std::pair<StateId, bool>> map_point_ids;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), map_point_positions);
    }

    std::map<StateId, Position> map_point_positions;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_UPDATES} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), map_point_updates);
    }

    std::vector<std::pair<StateId, Position>> map_point_updates;
};

template <>
class SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS> : public SharedMapMessageBase {
  public:
    SharedMapMessage() : SharedMapMessageBase{SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS} {};
    ~SharedMapMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SharedMapMessageBase>(this), map_point_ids);
    }

    std::set<StateId> map_point_ids;
};

} // namespace deco

CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::OBSERVATION_STORAGE>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::EDGE_UPDATES>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_MERGE_REQUESTS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::KEYFRAME_REQUESTS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::KEYFRAME_UPDATES>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::KEYFRAME_DELETION_REQUESTS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_REQUESTS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_UPDATES>)
CEREAL_REGISTER_TYPE(deco::SharedMapMessage<deco::SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>)
