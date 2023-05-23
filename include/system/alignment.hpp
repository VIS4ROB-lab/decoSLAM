//
// Created by philipp on 28.11.22.
//

#pragma once

#include <mutex>

#include "global.hpp"
#include "messages/alignment_messages.hpp"

namespace deco {
class Communicator;
class SharedMap;
class LocalStorage;

class Alignment {
  public:
    Alignment(const Id& id, StorageMutexHandler& mutex_handler);
    ~Alignment();

    auto setCommunicator(Communicator* communicator) -> void;
    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;

    auto submitMessage(std::unique_ptr<MessageBase>&& message) -> void;

    auto isAligned(const Id& id) const -> bool;
    auto addTransformation(const Id& id_A, const Id& id_B, const SE3& se3_A_B) -> void;
    auto getTransformationToWorld(const Id& id) -> SE3;

    auto getRootId() -> Id&;

  private:
    auto updateRoot_(AlignmentMessageBase& message) -> void;
    auto addNewChild_(AlignmentMessageBase& message) -> void;

    Id id_;
    Id root_id_;
    std::map<Id, SE3> root_alignment_transformations_;

    std::set<Id> aligned_ids_;

    StorageMutexHandler& mutex_handler_;
    Communicator* communicator_;
    SharedMap* shared_map_;
    LocalStorage* local_storage_;
};

} // namespace deco
