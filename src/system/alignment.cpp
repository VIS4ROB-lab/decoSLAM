//
// Created by philipp on 28.11.22.
//

#include <hyper/variables/groups/se3.hpp>

#include "messages/alignment_messages.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"

namespace deco {

Alignment::Alignment(const Id& id, StorageMutexHandler& mutex_handler)
    : id_{id},
      root_id_{id},
      mutex_handler_{mutex_handler} {}

Alignment::~Alignment() = default;

auto Alignment::setCommunicator(Communicator* communicator) -> void {
    communicator_ = communicator;
}

auto Alignment::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto Alignment::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto Alignment::submitMessage(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->type == MessageType::ALIGNMENT);
    auto& alignment_message = dynamic_cast<AlignmentMessageBase&>(*message);

    switch (alignment_message.subtype) {
        case AlignmentMessageType::UPDATE_ROOT:
            updateRoot_(alignment_message);
            break;
        case AlignmentMessageType::NEW_CHILD:
            addNewChild_(alignment_message);
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto Alignment::getRootId() -> Id& {
    return root_id_;
}

auto Alignment::isAligned(const Id& id) const -> bool {
    return id == root_id_ || root_alignment_transformations_.contains(id) || aligned_ids_.contains(id);
}

auto Alignment::addTransformation(const Id& id_A, const Id& id_B, const SE3& se3_A_B) -> void {
    DCHECK(id_A != id_B) << "Alignment IDs are identical.";
    DCHECK(id_A == root_id_ || id_B == root_id_) << "None of the alignment IDs corresponds to root.";
    const auto other_id = (id_A == root_id_ ? id_B : id_A);
    const auto se3_root_other = (id_A == root_id_) ? se3_A_B : se3_A_B.groupInverse();

    if (other_id < root_id_) {
        auto alignment_message = std::make_unique<AlignmentMessage<AlignmentMessageType::UPDATE_ROOT>>();
        alignment_message->receiver_id = root_id_;
        alignment_message->new_root_id = other_id;
        alignment_message->se3_newroot_receiver = se3_root_other.groupInverse();
        communicator_->send(std::move(alignment_message));
    } else {
        auto alignment_message = std::make_unique<AlignmentMessage<AlignmentMessageType::UPDATE_ROOT>>();
        alignment_message->receiver_id = other_id;
        alignment_message->new_root_id = root_id_;
        alignment_message->se3_newroot_receiver = se3_root_other;
        communicator_->send(std::move(alignment_message));
    }
}

auto Alignment::getTransformationToWorld(const Id& id) -> SE3 {
    DCHECK(root_alignment_transformations_.contains(id)) << "Transformation from " << id << " to " << root_id_ << " (current world id) does not exist.";
    return root_alignment_transformations_.find(id)->second;
}

auto Alignment::updateRoot_(AlignmentMessageBase& message) -> void {
    auto& query = dynamic_cast<AlignmentMessage<AlignmentMessageType::UPDATE_ROOT>&>(message);

    // Synchronized transform of transformations, local storage and shared map.
    if (query.new_root_id < root_id_) {
        auto lock = mutex_handler_.lockAllStorages();

        // Update all transformations and store new root.
        LOG(INFO) << id_ << ": Change of root " << root_id_ << " --> " << query.new_root_id;
        for (auto& [id, se3] : root_alignment_transformations_) {
            se3 = query.se3_newroot_receiver.groupPlus(se3);
        }
        auto [_, inserted] = root_alignment_transformations_.insert({root_id_, query.se3_newroot_receiver});
        DCHECK(inserted);

        local_storage_->applyTransformation(query.new_root_id, query.se3_newroot_receiver);
        shared_map_->applyTransformation(query.new_root_id, query.se3_newroot_receiver);

        // Register as new child.
        auto new_child_message = std::make_unique<AlignmentMessage<AlignmentMessageType::NEW_CHILD>>();
        new_child_message->receiver_id = query.new_root_id;
        new_child_message->new_child_id = id_;
        new_child_message->old_root_id = root_id_;
        communicator_->send(std::move(new_child_message));

        // Pass on the transformation to all aligned agents.
        for (const auto& id : aligned_ids_) {
            auto alignment_message = std::make_unique<AlignmentMessage<AlignmentMessageType::UPDATE_ROOT>>();
            alignment_message->receiver_id = id;
            alignment_message->new_root_id = query.new_root_id;
            alignment_message->se3_newroot_receiver = query.se3_newroot_receiver;
            communicator_->send(std::move(alignment_message));
        }
        aligned_ids_.clear(); // Aligned agents register directly with the root.

        root_id_ = query.new_root_id; // Replace the old root with the new one.
    } else {
        LOG(INFO) << id_ << "Ignoring transform to " << query.new_root_id << " sent from " << query.sender_id;
    }
}

auto Alignment::addNewChild_(AlignmentMessageBase& message) -> void {
    auto& query = dynamic_cast<AlignmentMessage<AlignmentMessageType::NEW_CHILD>&>(message);
    auto lock = mutex_handler_.lockAlignment();
    DCHECK(query.new_child_id > root_id_);
    if (id_ == root_id_) {
        LOG(INFO) << id_ << ": Add child " << query.new_child_id;
        auto [_, inserted] = aligned_ids_.insert(query.new_child_id);
        DCHECK(inserted);
    } else {
        DCHECK(aligned_ids_.empty());
        DCHECK(root_alignment_transformations_.contains(query.old_root_id));
        LOG(INFO) << id_ << "Relaying alignment from " << query.new_child_id << " to " << root_id_;
        auto alignment_message = std::make_unique<AlignmentMessage<AlignmentMessageType::UPDATE_ROOT>>();
        alignment_message->receiver_id = query.new_child_id;
        alignment_message->new_root_id = root_id_;
        alignment_message->se3_newroot_receiver = root_alignment_transformations_.find(query.old_root_id)->second;
        communicator_->send(std::move(alignment_message));
    }
}

} // namespace deco
