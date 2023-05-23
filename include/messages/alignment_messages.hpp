//
// Created by philipp on 28.11.22.
//

#pragma once

#include <cereal/types/polymorphic.hpp>

#include "messages/base.hpp"

namespace deco {
enum class AlignmentMessageType {
    UPDATE_ROOT,
    NEW_CHILD
};

template <>
class Message<MessageType::ALIGNMENT> : public MessageBase {
  public:
    explicit Message(const AlignmentMessageType& message_type) : MessageBase{MessageType::ALIGNMENT}, subtype{message_type} {};
    ~Message() override = default;

    auto getMessageCategory() -> MessageCategory override {
        return MessageCategory::LOOP_CLOSURE;
    }

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<MessageBase>(this), subtype);
    }

    AlignmentMessageType subtype; ///< Subtype of Alignment message.
};
using AlignmentMessageBase = Message<MessageType::ALIGNMENT>;
template <AlignmentMessageType> class AlignmentMessage;

template <>
class AlignmentMessage<AlignmentMessageType::UPDATE_ROOT> : public AlignmentMessageBase {
  public:
    AlignmentMessage() : AlignmentMessageBase{AlignmentMessageType::UPDATE_ROOT} {};
    ~AlignmentMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<AlignmentMessageBase>(this), new_root_id, se3_newroot_receiver);
    }

    Id new_root_id;           ///<
    SE3 se3_newroot_receiver; ///<
};

template <>
class AlignmentMessage<AlignmentMessageType::NEW_CHILD> : public AlignmentMessageBase {
  public:
    AlignmentMessage() : AlignmentMessageBase{AlignmentMessageType::NEW_CHILD} {};
    ~AlignmentMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<AlignmentMessageBase>(this), new_child_id, old_root_id);
    }

    Id new_child_id; ///<
    Id old_root_id;
};

} // namespace deco

CEREAL_REGISTER_TYPE(deco::AlignmentMessage<deco::AlignmentMessageType::UPDATE_ROOT>)
CEREAL_REGISTER_TYPE(deco::AlignmentMessage<deco::AlignmentMessageType::NEW_CHILD>)
