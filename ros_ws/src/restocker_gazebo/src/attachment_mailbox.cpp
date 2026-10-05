// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_mailbox.hpp"

#include <utility>

namespace restocker_gazebo
{

AttachmentMailbox::AttachmentMailbox(
  std::string simulator_epoch, std::size_t journal_capacity,
  std::optional<AttachmentIdentity> recovered_attachment,
  std::size_t detach_attempt_reserve)
: journal_(
    std::move(simulator_epoch), journal_capacity, std::move(recovered_attachment),
    detach_attempt_reserve)
{
}

msgs::AttachmentReply AttachmentMailbox::handle_set(const msgs::SetAttachmentRequest & message)
{
  AttachmentStatus error;
  const auto command = decode_command(message, error);
  std::scoped_lock lock(mutex_);
  if (!command) {
    return encode_reply(error_reply_locked(error));
  }
  if (command->expected_simulator_epoch != journal_.state().simulator_epoch) {
    return encode_reply(
      error_reply_locked(
      {
        AttachmentStatusCode::kSimulatorEpochChanged,
        "attachment command expected a different simulator epoch",
      }));
  }

  auto reply = journal_.submit(command->operation_id, command->request);
  if (reply.status.code == AttachmentStatusCode::kPending && !reply.replayed) {
    pending_ = *command;
  }
  return encode_reply(reply);
}

msgs::AttachmentReply AttachmentMailbox::handle_query(
  const msgs::QueryAttachmentRequest & message) const
{
  AttachmentStatus error;
  const auto query = decode_query(message, error);
  std::scoped_lock lock(mutex_);
  if (!query) {
    return encode_reply(error_reply_locked(error));
  }
  if (query->expected_simulator_epoch != journal_.state().simulator_epoch) {
    return encode_reply(
      error_reply_locked(
      {
        AttachmentStatusCode::kSimulatorEpochChanged,
        "attachment query expected a different simulator epoch",
      }));
  }
  return encode_reply(journal_.query(query->operation_id));
}

std::optional<DecodedAttachmentCommand> AttachmentMailbox::take_pending()
{
  std::scoped_lock lock(mutex_);
  auto result = std::move(pending_);
  pending_.reset();
  return result;
}

AttachmentJournalReply AttachmentMailbox::mark_validation_succeeded(
  std::string_view operation_id,
  std::optional<AttachmentFidelity> fidelity)
{
  std::scoped_lock lock(mutex_);
  return journal_.mark_validation_succeeded(operation_id, std::move(fidelity));
}

AttachmentJournalReply AttachmentMailbox::reject_before_mutation(
  std::string_view operation_id,
  AttachmentStatus rejection,
  std::optional<AttachmentFidelity> fidelity)
{
  std::scoped_lock lock(mutex_);
  return journal_.reject_before_mutation(operation_id, std::move(rejection), std::move(fidelity));
}

AttachmentJournalReply AttachmentMailbox::mark_mutation_started(std::string_view operation_id)
{
  std::scoped_lock lock(mutex_);
  return journal_.mark_mutation_started(operation_id);
}

AttachmentJournalReply AttachmentMailbox::mark_verification_succeeded(
  std::string_view operation_id,
  const AttachmentEvidence & evidence)
{
  std::scoped_lock lock(mutex_);
  return journal_.mark_verification_succeeded(operation_id, evidence);
}

AttachmentJournalReply AttachmentMailbox::mark_outcome_unknown(
  std::string_view operation_id,
  std::string detail)
{
  std::scoped_lock lock(mutex_);
  return journal_.mark_outcome_unknown(operation_id, std::move(detail));
}

AttachmentJournalReply AttachmentMailbox::mark_external_inconsistency(
  std::string detail,
  std::optional<AttachmentEvidence> evidence)
{
  std::scoped_lock lock(mutex_);
  return journal_.mark_external_inconsistency(std::move(detail), std::move(evidence));
}

void AttachmentMailbox::rotate_epoch(
  std::string simulator_epoch,
  std::optional<AttachmentIdentity> recovered_attachment)
{
  std::scoped_lock lock(mutex_);
  pending_.reset();
  journal_.rotate_epoch(std::move(simulator_epoch), std::move(recovered_attachment));
}

AttachmentPhysicalState AttachmentMailbox::state() const
{
  std::scoped_lock lock(mutex_);
  return journal_.state();
}

std::string AttachmentMailbox::simulator_epoch() const
{
  std::scoped_lock lock(mutex_);
  return journal_.state().simulator_epoch;
}

AttachmentJournalReply AttachmentMailbox::error_reply_locked(const AttachmentStatus & error) const
{
  auto result = journal_.query();
  result.status = error;
  result.replayed = false;
  result.operation.reset();
  return result;
}

}  // namespace restocker_gazebo
