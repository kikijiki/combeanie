// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <latch>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "restocker_task_executor/campaign_confirm_report.hpp"

namespace restocker_task_executor
{
namespace
{

using Source = CampaignConfirmReport::Source;

TEST(CampaignConfirmReport, FeedbackPublishesOnlyOnTheWorkerAndOnlyOnce)
{
  const auto worker = std::this_thread::get_id();
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {
      EXPECT_EQ(std::this_thread::get_id(), worker);
      reports.push_back(source);
    });
  std::thread executor([feedback = request.feedback_callback()]() {
      feedback();
      feedback();
    });
  executor.join();
  EXPECT_TRUE(reports.empty());
  request.poll();
  request.poll();
  request.finish(true);
  request.feedback_callback()();
  request.poll();
  EXPECT_EQ(reports, std::vector<Source>{Source::kFeedback});
}

TEST(CampaignConfirmReport, ResultBeforeLateFeedbackReportsOnceFromResult)
{
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {reports.push_back(source);});
  std::latch release_feedback{1};
  std::thread executor([feedback = request.feedback_callback(), &release_feedback]() {
      release_feedback.wait();
      feedback();
    });
  request.finish(true);
  release_feedback.count_down();
  executor.join();
  request.poll();
  request.finish(true);
  EXPECT_EQ(reports, std::vector<Source>{Source::kResult});
}

TEST(CampaignConfirmReport, TerminalDrainsUnpolledFeedbackBeforeAdvancing)
{
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {reports.push_back(source);});
  std::latch recorded{1};
  std::thread executor([feedback = request.feedback_callback(), &recorded]() {
      feedback();
      recorded.count_down();
    });
  recorded.wait();
  request.finish(true);
  executor.join();
  EXPECT_EQ(reports, std::vector<Source>{Source::kFeedback});
}

TEST(CampaignConfirmReport, TimeoutDrainsObservedConfirmWithoutInventingResultProof)
{
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {reports.push_back(source);});
  request.feedback_callback()();
  request.finish();
  request.finish(true);
  request.poll();
  EXPECT_EQ(reports, std::vector<Source>{Source::kFeedback});
}

TEST(CampaignConfirmReport, TimeoutRetiresBeforeLateFeedbackAndResultWithoutReplacement)
{
  // Both admission and result timeouts call finish() before returning to the campaign.
  // No next request is needed to fence the abandoned one.
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {reports.push_back(source);});
  std::latch callback_entered{1};
  std::latch release_feedback{1};
  std::thread executor([feedback = request.feedback_callback(), &callback_entered,
    &release_feedback]() {
      callback_entered.count_down();
      release_feedback.wait();
      feedback();
    });
  callback_entered.wait();
  request.finish();
  release_feedback.count_down();
  executor.join();
  request.poll();
  request.finish(true);
  EXPECT_TRUE(reports.empty());
}

TEST(CampaignConfirmReport, OldFeedbackAndResultCannotTakeTheNextRequestsSlot)
{
  std::vector<std::string> reports;
  CampaignConfirmReport old_request([&](Source) {reports.push_back("old");});
  std::latch release_old_feedback{1};
  std::thread executor([feedback = old_request.feedback_callback(), &release_old_feedback]() {
      release_old_feedback.wait();
      feedback();
    });
  old_request.finish();
  CampaignConfirmReport next_request([&](Source) {reports.push_back("next");});
  release_old_feedback.count_down();
  executor.join();
  old_request.poll();
  old_request.finish(true);
  next_request.poll();
  EXPECT_TRUE(reports.empty());
  next_request.feedback_callback()();
  next_request.poll();
  next_request.finish(true);
  EXPECT_EQ(reports, std::vector<std::string>{"next"});
}

TEST(CampaignConfirmReport, NeverConfirmedDoesNotFabricateAStatus)
{
  std::vector<Source> reports;
  CampaignConfirmReport request([&](Source source) {reports.push_back(source);});
  request.poll();
  request.finish();
  request.feedback_callback()();
  request.poll();
  EXPECT_TRUE(reports.empty());
}

TEST(CampaignConfirmReport, ExceptionalExitRetiresWithoutRetainingThePublisher)
{
  std::function<void()> late_feedback;
  int reports = 0;
  auto publisher_lifetime = std::make_shared<int>(0);
  const std::weak_ptr<int> weak_publisher = publisher_lifetime;
  try {
    CampaignConfirmReport request([publisher_lifetime, &reports](Source) {++reports;});
    late_feedback = request.feedback_callback();
    publisher_lifetime.reset();
    throw std::runtime_error("send/result failed");
  } catch (const std::runtime_error &) {
  }
  EXPECT_TRUE(weak_publisher.expired());
  late_feedback();
  CampaignConfirmReport next_request([&](Source) {++reports;});
  next_request.finish();
  EXPECT_EQ(reports, 0);
}

TEST(CampaignConfirmReport, WorkerCannotAdvanceBetweenClaimAndPublication)
{
  // Hold the sink after its report is claimed. Feedback cannot publish on an executor,
  // and the worker cannot move on until synchronous publication has returned.
  std::promise<std::function<void()>> callback;
  std::latch allow_poll{1};
  std::latch publication_entered{1};
  std::latch release_publication{1};
  std::latch worker_advanced{1};
  std::vector<std::string> events;
  std::thread worker([&]() {
      const auto owner = std::this_thread::get_id();
      CampaignConfirmReport request(
        [&](Source) {
          EXPECT_EQ(std::this_thread::get_id(), owner);
          publication_entered.count_down();
          release_publication.wait();
          events.push_back("confirm");
        });
      callback.set_value(request.feedback_callback());
      allow_poll.wait();
      request.poll();
      request.finish(true);
      events.push_back("transfer");
      worker_advanced.count_down();
    });
  callback.get_future().get()();
  EXPECT_FALSE(publication_entered.try_wait());
  allow_poll.count_down();
  publication_entered.wait();
  EXPECT_FALSE(worker_advanced.try_wait());
  release_publication.count_down();
  worker.join();
  EXPECT_EQ(events, (std::vector<std::string>{"confirm", "transfer"}));
}

}  // namespace
}  // namespace restocker_task_executor
