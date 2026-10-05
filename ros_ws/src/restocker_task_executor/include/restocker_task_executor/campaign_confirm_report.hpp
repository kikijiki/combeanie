// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <utility>

namespace restocker_task_executor
{

// One tray request's confirm evidence. Only feedback_callback()'s returned function runs on
// executor threads; all other methods and publication belong to the campaign worker. Keeping
// publication on that worker orders it before the next mode/cycle without sharing campaign
// counters or measurement maps with callbacks. A new request gets a new inbox, never a reset.
class CampaignConfirmReport final
{
public:
  enum class Source {kFeedback, kResult};

  explicit CampaignConfirmReport(std::function<void(Source)> publish)
  : publish_(std::move(publish)) {}

  ~CampaignConfirmReport()
  {
    // Also retire on exceptions, without attempting publication during stack unwinding.
    inbox_->store(State::kClosed);
  }

  CampaignConfirmReport(const CampaignConfirmReport &) = delete;
  CampaignConfirmReport & operator=(const CampaignConfirmReport &) = delete;

  [[nodiscard]] auto feedback_callback() const
  {
    return [inbox = inbox_]() {
             State expected = State::kOpen;
             (void)inbox->compare_exchange_strong(expected, State::kObserved);
           };
  }

  void poll()
  {
    if (inbox_->load() == State::kObserved) {
      report_once(Source::kFeedback);
    }
  }

  // Close atomically with taking pending evidence, then publish synchronously before the
  // worker advances. Feedback either precedes this exchange and is drained, or is inert.
  // A result may supply proof only while this request is still active.
  void finish(bool result_proves_confirm = false)
  {
    const State previous = inbox_->exchange(State::kClosed);
    if (previous == State::kObserved) {
      report_once(Source::kFeedback);
    } else if (previous == State::kOpen && result_proves_confirm) {
      report_once(Source::kResult);
    }
  }

private:
  enum class State {kOpen, kObserved, kClosed};

  void report_once(Source source)
  {
    if (!reported_) {
      reported_ = true;
      publish_(source);
    }
  }

  std::shared_ptr<std::atomic<State>> inbox_{std::make_shared<std::atomic<State>>(State::kOpen)};
  std::function<void(Source)> publish_;
  bool reported_{false};
};

}  // namespace restocker_task_executor
