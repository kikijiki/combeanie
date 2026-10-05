// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "restocker_gazebo/attachment_protocol.hpp"

namespace restocker_gazebo
{

// Thread boundary between Gazebo Transport callbacks and simulation update phases. Transport
// methods never expose the entity-component manager; update-thread methods consume immutable work.
class AttachmentMailbox
{
public:
  explicit AttachmentMailbox(
    std::string simulator_epoch, std::size_t journal_capacity,
    std::optional<AttachmentIdentity> recovered_attachment = std::nullopt,
    std::size_t detach_attempt_reserve = 2);

  [[nodiscard]] msgs::AttachmentReply handle_set(const msgs::SetAttachmentRequest & message);
  [[nodiscard]] msgs::AttachmentReply handle_query(
    const msgs::QueryAttachmentRequest & message) const;

  [[nodiscard]] std::optional<DecodedAttachmentCommand> take_pending();
  [[nodiscard]] AttachmentJournalReply mark_validation_succeeded(
    std::string_view operation_id,
    std::optional<AttachmentFidelity> fidelity = std::nullopt);
  [[nodiscard]] AttachmentJournalReply reject_before_mutation(
    std::string_view operation_id,
    AttachmentStatus rejection,
    std::optional<AttachmentFidelity> fidelity = std::nullopt);
  [[nodiscard]] AttachmentJournalReply mark_mutation_started(std::string_view operation_id);
  [[nodiscard]] AttachmentJournalReply mark_verification_succeeded(
    std::string_view operation_id,
    const AttachmentEvidence & evidence);
  [[nodiscard]] AttachmentJournalReply mark_outcome_unknown(
    std::string_view operation_id,
    std::string detail);
  [[nodiscard]] AttachmentJournalReply mark_external_inconsistency(
    std::string detail,
    std::optional<AttachmentEvidence> evidence = std::nullopt);

  void rotate_epoch(
    std::string simulator_epoch,
    std::optional<AttachmentIdentity> recovered_attachment = std::nullopt);
  [[nodiscard]] AttachmentPhysicalState state() const;
  [[nodiscard]] std::string simulator_epoch() const;

private:
  // The caller must hold mutex_; the reply snapshots the journal atomically with the error.
  [[nodiscard]] AttachmentJournalReply error_reply_locked(const AttachmentStatus & error) const;

  mutable std::mutex mutex_;
  AttachmentJournal journal_;
  std::optional<DecodedAttachmentCommand> pending_;
};

}  // namespace restocker_gazebo
