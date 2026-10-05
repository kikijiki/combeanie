// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Diagnostic for the advisory client.
//
// Asks a real backend one of the reproduced failure questions through the same client the
// coordinator uses, and prints the answer verbatim next to the validator's verdict. It captured
// the responses under test/recorded_responses and shows what a model says about a failure without
// running the robot. Test-time only: nothing in the runtime depends on it, and it is not
// installed.

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "restocker_reasoner/http_recovery_advisor.hpp"
#include "restocker_reasoner/recovery_advice.hpp"
#include "restocker_reasoner/strict_json.hpp"

#include "recorded_failure_cases.hpp"

namespace
{

void usage()
{
  std::cerr <<
    "usage: reasoner_probe --case <name> [--host H] [--port P] [--model M]\n"
    "                      [--timeout-ms N] [--max-tokens N] [--min-confidence C]\n"
    "                      [--no-constrain] [--list]\n"
    "\n"
    "Asks a running OpenAI-compatible backend one reproduced failure question and prints the\n"
    "answer verbatim with the validator's verdict. Never required by the runtime.\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  using restocker_reasoner::HttpRecoveryAdvisor;
  using restocker_reasoner::HttpRecoveryAdvisorConfig;
  using restocker_reasoner::RecoveryAdviceCompletion;

  HttpRecoveryAdvisorConfig config;
  config.deadline = std::chrono::milliseconds(60000);
  std::string case_name;
  double minimum_confidence = 0.5;

  const std::vector<std::string> arguments(argv + 1, argv + argc);
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const auto & argument = arguments[index];
    const auto next = [&arguments, &index]() -> std::string {
      return index + 1U < arguments.size() ? arguments[++index] : std::string();
    };
    if (argument == "--list") {
      for (const auto & recorded : restocker_reasoner::recorded_failure_cases()) {
        std::cout << recorded.name << '\n';
      }
      return 0;
    } else if (argument == "--case") {
      case_name = next();
    } else if (argument == "--host") {
      config.host = next();
    } else if (argument == "--port") {
      config.port = static_cast<std::uint16_t>(std::atoi(next().c_str()));
    } else if (argument == "--model") {
      config.model = next();
    } else if (argument == "--max-tokens") {
      config.maximum_output_tokens = std::atoi(next().c_str());
    } else if (argument == "--timeout-ms") {
      config.deadline = std::chrono::milliseconds(std::atoi(next().c_str()));
    } else if (argument == "--min-confidence") {
      minimum_confidence = std::atof(next().c_str());
    } else if (argument == "--no-constrain") {
      config.constrained_decoding = false;
    } else {
      usage();
      return 2;
    }
  }

  const auto query = restocker_reasoner::recorded_failure_case(case_name);
  if (!query) {
    std::cerr << "unknown case \"" << case_name << "\"; --list names them\n";
    return 2;
  }

  std::mutex mutex;
  std::condition_variable arrived;
  std::optional<RecoveryAdviceCompletion> completion;

  HttpRecoveryAdvisor advisor(config);
  const auto submitted = advisor.submit(
    *query, [&](RecoveryAdviceCompletion result) {
      {
        const std::lock_guard<std::mutex> guard(mutex);
        completion = std::move(result);
      }
      arrived.notify_all();
    });
  if (!submitted) {
    std::cerr << "the advisory client refused the question: " << submitted.detail << '\n';
    return 1;
  }

  {
    std::unique_lock<std::mutex> guard(mutex);
    arrived.wait_for(
      guard, config.deadline + std::chrono::seconds(5),
      [&completion]() {return completion.has_value();});
  }
  advisor.shutdown();

  if (!completion) {
    std::cerr << "the advisory client never completed\n";
    return 1;
  }

  std::cout << "case: " << case_name << '\n';
  std::cout << "backend: " << completion->backend << '\n';
  std::cout << "latency_ms: " << completion->latency.count() << '\n';
  std::cout << "responded: " << (completion->responded ? "true" : "false") << '\n';
  if (!completion->transport_detail.empty()) {
    std::cout << "transport_detail: " << completion->transport_detail << '\n';
  }
  std::cout << "query: " << restocker_reasoner::render_recovery_query_json(*query) << '\n';
  std::cout << "response_verbatim_begin\n" << completion->document <<
    "\nresponse_verbatim_end\n";

  if (!completion->responded) {
    std::cout << "verdict: no recommendation\n";
    return 0;
  }
  const auto parsed = restocker_reasoner::parse_recovery_advice(
    completion->document, *query, minimum_confidence);
  if (!parsed.advice) {
    std::cout << "verdict: rejected -- " << parsed.rejection << '\n';
    return 0;
  }
  std::cout << "verdict: accepted\n";
  std::cout << "failure_class: " <<
    restocker_reasoner::recovery_failure_class_name(parsed.advice->failure_class) << '\n';
  std::cout << "recommended_primitive: " <<
    (parsed.advice->recommended_primitive ?
  restocker_reasoner::recovery_primitive_name(*parsed.advice->recommended_primitive) :
  "null") << '\n';
  std::cout << "confidence: " << parsed.advice->confidence << '\n';
  std::cout << "deterministic_primitive: " <<
    restocker_reasoner::recovery_primitive_name(query->deterministic_primitive) << '\n';
  return 0;
}
