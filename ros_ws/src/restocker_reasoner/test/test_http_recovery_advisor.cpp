// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "restocker_reasoner/http_recovery_advisor.hpp"
#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

using namespace std::chrono_literals;

[[nodiscard]] RecoveryQuery question()
{
  RecoveryQuery query;
  query.request_id = "a1b2c3d4";
  query.goal_generation = 2U;
  query.failed_segment = "insert";
  query.motion_outcome = "planning_failed";
  query.task_state = "PlanInsert";
  query.observed_detail = "no plan";
  query.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
  query.permitted_primitives = {
    RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
  return query;
}

// Collects exactly one completion so a test can wait for it without spinning.
class CompletionSink
{
public:
  void deposit(RecoveryAdviceCompletion completion)
  {
    {
      const std::lock_guard<std::mutex> guard(mutex_);
      completion_ = std::move(completion);
    }
    arrived_.notify_all();
  }

  [[nodiscard]] std::optional<RecoveryAdviceCompletion> wait(std::chrono::milliseconds bound)
  {
    std::unique_lock<std::mutex> guard(mutex_);
    arrived_.wait_for(guard, bound, [this]() {return completion_.has_value();});
    return completion_;
  }

private:
  std::mutex mutex_;
  std::condition_variable arrived_;
  std::optional<RecoveryAdviceCompletion> completion_;
};

// A listening socket that accepts a connection and then says nothing, so the client's own
// deadline is the only thing that can end the call.
class SilentListener
{
public:
  SilentListener()
  {
    descriptor_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (descriptor_ < 0) {
      return;
    }
    ::sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(descriptor_, reinterpret_cast<::sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(descriptor_, 4) != 0)
    {
      ::close(descriptor_);
      descriptor_ = -1;
      return;
    }
    ::socklen_t length = sizeof(address);
    if (::getsockname(descriptor_, reinterpret_cast<::sockaddr *>(&address), &length) != 0) {
      ::close(descriptor_);
      descriptor_ = -1;
      return;
    }
    port_ = ::ntohs(address.sin_port);
  }

  ~SilentListener()
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
  }

  SilentListener(const SilentListener &) = delete;
  SilentListener & operator=(const SilentListener &) = delete;
  SilentListener(SilentListener &&) = delete;
  SilentListener & operator=(SilentListener &&) = delete;

  [[nodiscard]] bool listening() const {return descriptor_ >= 0;}
  [[nodiscard]] std::uint16_t port() const {return port_;}

private:
  int descriptor_{-1};
  std::uint16_t port_{0U};
};

}  // namespace

TEST(HttpRequestFraming, CarriesTheBodyItAdvertises)
{
  HttpRecoveryAdvisorConfig config;
  config.host = "127.0.0.1";
  config.port = 8080U;
  config.path = "/v1/chat/completions";
  const auto request = build_http_post_request(config, "{\"a\":1}");
  EXPECT_TRUE(request.starts_with("POST /v1/chat/completions HTTP/1.1\r\n"));
  EXPECT_NE(request.find("Host: 127.0.0.1:8080\r\n"), std::string::npos);
  EXPECT_NE(request.find("Content-Length: 7\r\n"), std::string::npos);
  EXPECT_NE(request.find("Connection: close\r\n"), std::string::npos);
  EXPECT_TRUE(request.ends_with("\r\n\r\n{\"a\":1}"));
}

TEST(HttpResponseFraming, ReadsAContentLengthBody)
{
  const auto parts = parse_http_response(
    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 7\r\n\r\n{\"a\":1}",
    65536U);
  ASSERT_TRUE(parts.error.empty()) << parts.error;
  EXPECT_EQ(parts.status_code, 200);
  EXPECT_EQ(parts.body, "{\"a\":1}");
}

// llama.cpp answers chunked, so this is the framing the real backend actually uses.
TEST(HttpResponseFraming, ReadsAChunkedBody)
{
  const auto parts = parse_http_response(
    "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n4\r\n{\"a\"\r\n3;x=1\r\n:1}\r\n0\r\n\r\n",
    65536U);
  ASSERT_TRUE(parts.error.empty()) << parts.error;
  EXPECT_EQ(parts.body, "{\"a\":1}");
}

TEST(HttpResponseFraming, RefusesWhatItCannotFrameRatherThanGuessing)
{
  EXPECT_FALSE(parse_http_response("", 65536U).error.empty());
  EXPECT_FALSE(parse_http_response("garbage\r\n\r\nbody", 65536U).error.empty());
  EXPECT_FALSE(parse_http_response("HTTP/1.1 zzz OK\r\n\r\n", 65536U).error.empty());
  EXPECT_FALSE(
    parse_http_response(
      "HTTP/1.1 200 OK\r\nContent-Length: many\r\n\r\nbody", 65536U).error.empty());
  EXPECT_FALSE(
    parse_http_response(
      "HTTP/1.1 200 OK\r\nContent-Length: 99\r\n\r\nshort", 65536U).error.empty());
  EXPECT_FALSE(
    parse_http_response(
      "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nzz\r\n", 65536U).error.empty());
  EXPECT_FALSE(
    parse_http_response(
      "HTTP/1.1 200 OK\r\nContent-Length: 7\r\n\r\n{\"a\":1}", 3U).error.empty()) <<
    "past the byte bound";
}

// A backend-chosen chunk size can be near SIZE_MAX. Bounds written as additions wrap, the checks
// pass, the cursor moves backwards, and the worker spins forever in a call no deadline can end,
// blocking coordinator shutdown through the destructor's join. Every bound in the decoder is
// therefore a subtraction.
TEST(HttpResponseFraming, RefusesAChunkSizeThatWouldWrapItsOwnBounds)
{
  const std::string body =
    std::string("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n") +
    "14\r\n" + std::string(20U, 'A') + "\r\n" + "FFFFFFFFFFFFFFEC\r\n";
  const auto parts = parse_http_response(body, 65536U);
  EXPECT_FALSE(parts.error.empty());

  // The same shape without the wrap, so the test above is failing for the reason it says.
  EXPECT_FALSE(
    parse_http_response(
      std::string("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n") +
      "14\r\n" + std::string(20U, 'A') + "\r\n" + "FFFF\r\n", 65536U).error.empty());
}

TEST(HttpResponseFraming, RequiresTheCrlfThatTerminatesAChunkBody)
{
  EXPECT_FALSE(
    parse_http_response(
      "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n4\r\n{\"a\"XX0\r\n\r\n",
      65536U).error.empty());
}

// A header value that is only whitespace once trimmed. Different character sets at the two ends
// used to underflow the length.
TEST(HttpResponseFraming, HandlesAHeaderValueThatIsOnlyWhitespace)
{
  const auto parts = parse_http_response(
    "HTTP/1.1 200 OK\r\nContent-Length: \r\nX: \r\n\r\nbody", 65536U);
  EXPECT_FALSE(parts.error.empty()) << "an empty Content-Length is not a number";
}

TEST(HttpResponseFraming, ReportsANonSuccessStatus)
{
  const auto parts = parse_http_response(
    "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n", 65536U);
  ASSERT_TRUE(parts.error.empty()) << parts.error;
  EXPECT_EQ(parts.status_code, 503);
}

TEST(ChatCompletionBody, IsValidJsonAndCarriesTheClosedPrimitiveSet)
{
  HttpRecoveryAdvisorConfig config;
  config.model = "a-model";
  const auto body = build_chat_completion_body(question(), config);
  const auto parsed = parse_strict_json(body, JsonParseLimits{65536U, 16U});
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_EQ(parsed.value->member("model")->as_string(), "a-model");
  ASSERT_NE(parsed.value->member("messages"), nullptr);
  EXPECT_EQ(parsed.value->member("messages")->as_array().size(), 2U);
  EXPECT_NE(parsed.value->member("response_format"), nullptr);

  config.constrained_decoding = false;
  const auto unconstrained = build_chat_completion_body(question(), config);
  const auto reparsed = parse_strict_json(unconstrained, JsonParseLimits{65536U, 16U});
  ASSERT_TRUE(reparsed.value) << reparsed.error;
  EXPECT_EQ(reparsed.value->member("response_format"), nullptr);
}

TEST(ChatCompletionEnvelope, ExtractsTheContentAndRefusesEverythingElse)
{
  const auto found = extract_chat_completion_content(
    R"({"choices":[{"message":{"role":"assistant","content":"{}"}}],"usage":{"total_tokens":9}})");
  ASSERT_TRUE(found.error.empty()) << found.error;
  EXPECT_EQ(found.content, "{}");

  EXPECT_FALSE(extract_chat_completion_content("not json").error.empty());
  EXPECT_FALSE(extract_chat_completion_content(R"({"choices":[]})").error.empty());
  EXPECT_FALSE(extract_chat_completion_content(R"({"choices":[{}]})").error.empty());
  EXPECT_FALSE(
    extract_chat_completion_content(
      R"({"choices":[{"message":{"content":""}}]})").error.empty()) <<
    "a model that spent its whole budget thinking returns an empty content";
  EXPECT_FALSE(
    extract_chat_completion_content(
      R"({"choices":[{"message":{"content":42}}]})").error.empty());
}

TEST(HttpRecoveryAdvisor, RefusesAQuestionItCouldNotAnswerMeaningfully)
{
  HttpRecoveryAdvisorConfig config;
  config.port = 1U;
  HttpRecoveryAdvisor advisor(config);

  EXPECT_EQ(
    advisor.submit(question(), {}).status, RecoveryAdviceSubmitStatus::kInvalidRequest);

  auto without_id = question();
  without_id.request_id.clear();
  EXPECT_EQ(
    advisor.submit(without_id, [](RecoveryAdviceCompletion) {}).status,
    RecoveryAdviceSubmitStatus::kInvalidRequest);

  auto without_permission = question();
  without_permission.permitted_primitives.clear();
  EXPECT_EQ(
    advisor.submit(without_permission, [](RecoveryAdviceCompletion) {}).status,
    RecoveryAdviceSubmitStatus::kInvalidRequest);

  advisor.shutdown();
  EXPECT_EQ(
    advisor.submit(question(), [](RecoveryAdviceCompletion) {}).status,
    RecoveryAdviceSubmitStatus::kUnavailable);
  // Shutting down twice must be safe; the destructor calls it again.
  advisor.shutdown();
}

// The client is optional, so nothing listening is the ordinary case: it must complete promptly
// with no document and a reason.
TEST(HttpRecoveryAdvisor, AnAbsentBackendCompletesAsNoRecommendation)
{
  HttpRecoveryAdvisorConfig config;
  // Port 1 on loopback: refused immediately in every environment this runs in, and if it is
  // firewalled instead, the deadline ends the call with the same completion.
  config.port = 1U;
  config.deadline = 1500ms;
  config.consecutive_failures_before_open = 2U;
  HttpRecoveryAdvisor advisor(config);

  CompletionSink sink;
  ASSERT_TRUE(
    static_cast<bool>(
      advisor.submit(
        question(), [&sink](RecoveryAdviceCompletion completion) {
          sink.deposit(std::move(completion));
        })));
  const auto completion = sink.wait(5s);
  ASSERT_TRUE(completion);
  EXPECT_FALSE(completion->responded);
  EXPECT_TRUE(completion->document.empty());
  EXPECT_FALSE(completion->transport_detail.empty());
  EXPECT_EQ(completion->request_id, "a1b2c3d4");
  EXPECT_EQ(completion->goal_generation, 2U);
  EXPECT_FALSE(completion->backend.empty());
  advisor.shutdown();
}

TEST(HttpRecoveryAdvisor, StopsCallingABackendThatKeepsFailing)
{
  HttpRecoveryAdvisorConfig config;
  config.port = 1U;
  config.deadline = 1500ms;
  config.consecutive_failures_before_open = 2U;
  config.circuit_cooldown = 60s;
  HttpRecoveryAdvisor advisor(config);

  for (std::size_t attempt = 0U; attempt < 2U; ++attempt) {
    CompletionSink sink;
    const auto submitted = advisor.submit(
      question(), [&sink](RecoveryAdviceCompletion completion) {
        sink.deposit(std::move(completion));
      });
    ASSERT_TRUE(static_cast<bool>(submitted)) << submitted.detail;
    ASSERT_TRUE(sink.wait(5s));
  }
  EXPECT_GE(advisor.consecutive_failures(), 2U);
  EXPECT_FALSE(advisor.ready());
  const auto refused = advisor.submit(question(), [](RecoveryAdviceCompletion) {});
  EXPECT_EQ(refused.status, RecoveryAdviceSubmitStatus::kUnavailable);
  advisor.shutdown();
}

// Shutdown races the worker's own wait. If the store that ends the wait is not published under
// the mutex the worker evaluates its predicate with, a shutdown in that window is missed and the
// join never returns. Repetition is the only way to catch it; each iteration starts and stops a
// fresh thread immediately, which is where the window is.
TEST(HttpRecoveryAdvisor, ShutsDownPromptlyHoweverTheWorkerIsRacedIntoIt)
{
  for (std::size_t attempt = 0U; attempt < 64U; ++attempt) {
    HttpRecoveryAdvisorConfig config;
    config.port = 1U;
    HttpRecoveryAdvisor advisor(config);
    advisor.shutdown();
  }
}

TEST(HttpRecoveryAdvisor, ABackendThatNeverAnswersEndsAtTheDeadline)
{
  SilentListener listener;
  ASSERT_TRUE(listener.listening());

  HttpRecoveryAdvisorConfig config;
  config.port = listener.port();
  config.deadline = 400ms;
  HttpRecoveryAdvisor advisor(config);

  CompletionSink sink;
  ASSERT_TRUE(
    static_cast<bool>(
      advisor.submit(
        question(), [&sink](RecoveryAdviceCompletion completion) {
          sink.deposit(std::move(completion));
        })));
  const auto completion = sink.wait(10s);
  ASSERT_TRUE(completion);
  EXPECT_FALSE(completion->responded);
  EXPECT_NE(completion->transport_detail.find("deadline"), std::string::npos)
    << completion->transport_detail;
  // The deadline is hard: the call cannot outlive it by more than the poll slice and the wait
  // that noticed it.
  EXPECT_LT(completion->latency, 5s);
  advisor.shutdown();
}

}  // namespace restocker_reasoner
