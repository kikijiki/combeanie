// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "restocker_reasoner/fake_recovery_advisor.hpp"
#include "restocker_reasoner/recovery_advisor_port.hpp"

namespace restocker_reasoner
{

namespace
{

[[nodiscard]] RecoveryQuery question(std::string request_id, std::uint64_t generation)
{
  RecoveryQuery query;
  query.request_id = std::move(request_id);
  query.goal_generation = generation;
  query.permitted_primitives = {RecoveryPrimitive::kAbandonTask};
  return query;
}

[[nodiscard]] RecoveryAdviceCompletion answer(std::string request_id, std::uint64_t generation)
{
  RecoveryAdviceCompletion completion;
  completion.request_id = std::move(request_id);
  completion.goal_generation = generation;
  completion.responded = true;
  completion.document = "{}";
  return completion;
}

}  // namespace

TEST(RecoveryAdviceMailbox, HandsBackOnlyTheAnswerToTheQuestionBeingAsked)
{
  RecoveryAdviceMailbox mailbox;
  EXPECT_FALSE(mailbox.occupied());
  EXPECT_FALSE(mailbox.take("one", 1U));

  mailbox.deposit(answer("one", 1U));
  EXPECT_TRUE(mailbox.occupied());
  const auto taken = mailbox.take("one", 1U);
  ASSERT_TRUE(taken);
  EXPECT_EQ(taken->document, "{}");
  EXPECT_FALSE(mailbox.occupied()) << "taking empties the slot";
}

TEST(RecoveryAdviceMailbox, DiscardsAnAnswerToASupersededQuestion)
{
  RecoveryAdviceMailbox mailbox;
  mailbox.deposit(answer("one", 1U));
  EXPECT_FALSE(mailbox.take("two", 1U)) << "a different request";
  // The mismatched answer is discarded, so it cannot resurface against another question.
  EXPECT_FALSE(mailbox.occupied());

  mailbox.deposit(answer("one", 1U));
  EXPECT_FALSE(mailbox.take("one", 2U)) << "a different goal generation";
  EXPECT_FALSE(mailbox.occupied());
}

TEST(RecoveryAdviceMailbox, KeepsOnlyTheNewestAnswer)
{
  RecoveryAdviceMailbox mailbox;
  mailbox.deposit(answer("one", 1U));
  mailbox.deposit(answer("two", 1U));
  EXPECT_FALSE(mailbox.take("one", 1U));
}

TEST(RecoveryAdviceMailbox, ClearsOnRequest)
{
  RecoveryAdviceMailbox mailbox;
  mailbox.deposit(answer("one", 1U));
  mailbox.clear();
  EXPECT_FALSE(mailbox.occupied());
}

TEST(FakeRecoveryAdvisor, CompletesThroughTheMailboxLikeARealBackendWould)
{
  const auto mailbox = std::make_shared<RecoveryAdviceMailbox>();
  FakeRecoveryAdvisor advisor;

  const auto submitted = advisor.submit(
    question("one", 3U), [mailbox](RecoveryAdviceCompletion completion) {
      mailbox->deposit(std::move(completion));
    });
  ASSERT_TRUE(static_cast<bool>(submitted)) << submitted.detail;
  EXPECT_TRUE(advisor.outstanding());
  EXPECT_FALSE(mailbox->occupied()) << "no answer before the backend gives one";

  advisor.respond("{\"a\":1}");
  const auto taken = mailbox->take("one", 3U);
  ASSERT_TRUE(taken);
  EXPECT_TRUE(taken->responded);
  EXPECT_EQ(taken->document, "{\"a\":1}");
}

TEST(FakeRecoveryAdvisor, NotRespondingIsAnOrdinaryCompletion)
{
  const auto mailbox = std::make_shared<RecoveryAdviceMailbox>();
  FakeRecoveryAdvisor advisor;
  ASSERT_TRUE(
    static_cast<bool>(
      advisor.submit(
        question("one", 1U), [mailbox](RecoveryAdviceCompletion completion) {
          mailbox->deposit(std::move(completion));
        })));

  advisor.respond_with_nothing("the advisory call passed its deadline");
  const auto taken = mailbox->take("one", 1U);
  ASSERT_TRUE(taken);
  EXPECT_FALSE(taken->responded);
  EXPECT_TRUE(taken->document.empty());
  EXPECT_EQ(taken->transport_detail, "the advisory call passed its deadline");
}

TEST(FakeRecoveryAdvisor, RefusesASecondOutstandingQuestion)
{
  FakeRecoveryAdvisor advisor;
  ASSERT_TRUE(
    static_cast<bool>(
      advisor.submit(question("one", 1U), [](RecoveryAdviceCompletion) {})));
  const auto second = advisor.submit(question("two", 1U), [](RecoveryAdviceCompletion) {});
  EXPECT_FALSE(static_cast<bool>(second));
  EXPECT_EQ(second.status, RecoveryAdviceSubmitStatus::kBusy);
}

}  // namespace restocker_reasoner
