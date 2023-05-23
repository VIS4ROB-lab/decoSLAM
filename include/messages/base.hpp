//
// Created by philipp on 09.08.22.
//

#pragma once
#include <cereal/types/base_class.hpp>

#include "global.hpp"

namespace deco {
enum class MessageType {
    PLACE_RECOGNITION,
    ALIGNMENT,
    SHARED_MAP,
    LOCAL_STORAGE,
    SYSTEM
};

enum class MessageCategory {
    LOOP_CLOSURE,
    MAP_TRANSFER,
    OPTIMIZATION,
    SYSTEM
};

class MessageBase {
  public:
    explicit MessageBase(const MessageType& message_type) : type{message_type} {};
    virtual ~MessageBase() = default;

    virtual auto getMessageCategory() -> MessageCategory = 0;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(sender_id, receiver_id, type);
    }

    Id sender_id;
    Id receiver_id;
    MessageType type;
};

template <MessageType> class Message;

} // namespace deco
