// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

// Refuse to run a ROS-graph gtest on a domain nobody leased.
//
// These tests register fixed action, service and topic names. On the shell's default domain the
// same test built in another worktree answers this one's goals and publishes into its buffers.
// The result reads as a load flake, because the machine is loaded exactly when several agents run
// package suites at once. cmake/restocker_test_isolation.cmake routes every gtest through the
// domain-lease runner, which stamps RESTOCKER_ISOLATION_TAG. A process without that tag was not
// leased a domain, so it fails before it creates a node instead of producing a result that
// depends on who else is running.
//
// To run a binary by hand, go through ctest, or export a free ROS_DOMAIN_ID and any non-empty
// RESTOCKER_ISOLATION_TAG yourself.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

namespace restocker_task_executor::test
{

class RosDomainLeaseGuard : public ::testing::Environment
{
public:
  void SetUp() override
  {
    const char * tag = std::getenv("RESTOCKER_ISOLATION_TAG");
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    ASSERT_TRUE(tag != nullptr && *tag != '\0')
      << "RESTOCKER_ISOLATION_TAG is unset: this ROS-graph test was not started through the "
      << "domain-lease runner (scripts/run_isolated_test.py), so it would share a ROS domain "
      << "with every other workspace's copy of itself";
    ASSERT_TRUE(domain != nullptr && *domain != '\0' && std::string(domain) != "0")
      << "ROS_DOMAIN_ID is unset or 0 despite an isolation tag: the leased domain did not reach "
      << "this process";
  }
};

// Registered once per test binary, before main() runs the tests.
inline ::testing::Environment * const kRosDomainLeaseGuard =
  ::testing::AddGlobalTestEnvironment(new RosDomainLeaseGuard());

}  // namespace restocker_task_executor::test
