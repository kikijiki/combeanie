// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <string>

#include "restocker_task_executor/recovery_classification.hpp"

namespace
{

using restocker_task_executor::classify_recovery;
using restocker_task_executor::RecoveryClassification;
using restocker_task_executor::RecoveryClass;
using restocker_task_executor::RecoveryContext;
using restocker_task_executor::RecoveryEvidence;
using restocker_task_executor::recovery_classification_receipt;

// The context everything first-hand evidence establishes: stopped, held-state known,
// re-observable, nothing irreversible. The starting point every rule below deviates from.
RecoveryContext established_context()
{
  RecoveryContext context;
  context.arm_stop = RecoveryEvidence::kEstablished;
  context.held_state = RecoveryEvidence::kEstablished;
  context.world_reobservable = true;
  context.cause = "pre-grasp planning refused the start state";
  return context;
}

TEST(RecoveryClassification, EstablishedContextIsRecoverable)
{
  const auto classification = classify_recovery(established_context());
  EXPECT_EQ(classification.klass, RecoveryClass::kRecoverable);
  EXPECT_EQ(classification.reason, "nothing irreversible happened");
}

TEST(RecoveryClassification, DefaultContextFailsClosedToUnsafe)
{
  // The spec's fail-closed default: nothing established, nothing may be retried blindly.
  const RecoveryContext context;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
}

TEST(RecoveryClassification, UnknownArmStopIsUnsafe)
{
  auto context = established_context();
  context.arm_stop = RecoveryEvidence::kUnknown;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(classification.reason, "the arm's stopped state is not established");
}

TEST(RecoveryClassification, ArmNotStoppedIsUnsafe)
{
  auto context = established_context();
  context.arm_stop = RecoveryEvidence::kNotEstablished;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(classification.reason, "the arm has not reached a verified stop");
}

TEST(RecoveryClassification, UnknownHeldStateIsUnsafe)
{
  auto context = established_context();
  context.held_state = RecoveryEvidence::kUnknown;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(classification.reason, "the held-object state is not established");
}

TEST(RecoveryClassification, InconsistentHeldStateIsUnsafe)
{
  auto context = established_context();
  context.held_state = RecoveryEvidence::kNotEstablished;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(
    classification.reason, "the held-object state is inconsistent with the task");
}

TEST(RecoveryClassification, ContactBeatsEverythingElse)
{
  auto context = established_context();
  context.contact_or_collision = true;
  context.arm_stop = RecoveryEvidence::kUnknown;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(classification.reason, "real contact or collision occurred");
}

TEST(RecoveryClassification, PersistentHardwareFaultIsUnsafe)
{
  auto context = established_context();
  context.persistent_hardware_fault = true;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(
    classification.reason,
    "a hardware/controller fault persists after re-initialisation");
}

TEST(RecoveryClassification, UnobservableWorldIsUnsafe)
{
  auto context = established_context();
  context.world_reobservable = false;
  const auto classification = classify_recovery(context);
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_EQ(classification.reason, "the world state is not re-observable");
}

TEST(RecoveryClassification, ReceiptCarriesClassReasonAndCause)
{
  auto context = established_context();
  context.held_state = RecoveryEvidence::kUnknown;
  context.cause = "attach reconciliation failed at the boundary";
  const auto classification = classify_recovery(context);
  const std::string receipt = recovery_classification_receipt(classification, context.cause);
  EXPECT_EQ(
    receipt,
    "recovery classification: UNSAFE (the held-object state is not established): "
    "attach reconciliation failed at the boundary");
}

TEST(RecoveryClassification, RecoverableReceiptNamesTheClass)
{
  const auto classification = classify_recovery(established_context());
  const std::string receipt =
    recovery_classification_receipt(classification, established_context().cause);
  EXPECT_EQ(
    receipt,
    "recovery classification: RECOVERABLE (nothing irreversible happened): "
    "pre-grasp planning refused the start state");
}

TEST(RecoveryClassification, EmptyCauseOmitsTheSuffix)
{
  const auto classification = classify_recovery(established_context());
  const std::string receipt = recovery_classification_receipt(classification, "");
  EXPECT_EQ(receipt, "recovery classification: RECOVERABLE (nothing irreversible happened)");
}

}  // namespace
