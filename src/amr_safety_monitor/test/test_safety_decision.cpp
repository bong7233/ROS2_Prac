// Unit tests for the pure safety-gate decision logic.
#include <gtest/gtest.h>

#include "amr_safety_monitor/safety_decision.hpp"

using amr_safety_monitor::evaluateSafetyDecision;
using amr_safety_monitor::SafetyInputs;

namespace
{
SafetyInputs clearInputs()
{
  return SafetyInputs{false, false, false, false, false, 0U, true, true, false};
}
}  // namespace

TEST(SafetyDecision, AllClearAllowsCommand)
{
  const auto d = evaluateSafetyDecision(clearInputs());
  EXPECT_TRUE(d.command_allowed);
  EXPECT_EQ(d.reason, "clear");
}

TEST(SafetyDecision, EstopBlocksCommand)
{
  auto in = clearInputs();
  in.estop_active = true;
  const auto d = evaluateSafetyDecision(in);
  EXPECT_FALSE(d.command_allowed);
  EXPECT_EQ(d.reason, "estop active");
}

TEST(SafetyDecision, CommandTimeoutBlocks)
{
  auto in = clearInputs();
  in.command_timeout = true;
  const auto d = evaluateSafetyDecision(in);
  EXPECT_FALSE(d.command_allowed);
  EXPECT_EQ(d.reason, "cmd_vel timeout");
}

TEST(SafetyDecision, MotorFaultIncludesCode)
{
  auto in = clearInputs();
  in.motor_fault = true;
  in.motor_fault_code = 2310U;
  const auto d = evaluateSafetyDecision(in);
  EXPECT_FALSE(d.command_allowed);
  EXPECT_EQ(d.reason, "motor fault 2310");
}

TEST(SafetyDecision, MotorDisabledOnlyWhenRequired)
{
  auto in = clearInputs();
  in.motor_enabled = false;
  in.require_motor_enabled = false;
  EXPECT_TRUE(evaluateSafetyDecision(in).command_allowed);

  in.require_motor_enabled = true;
  const auto d = evaluateSafetyDecision(in);
  EXPECT_FALSE(d.command_allowed);
  EXPECT_EQ(d.reason, "motor disabled");
}

TEST(SafetyDecision, MultipleReasonsAreJoinedInOrder)
{
  auto in = clearInputs();
  in.command_timeout = true;
  in.estop_active = true;
  in.communication_fault = true;
  const auto d = evaluateSafetyDecision(in);
  EXPECT_FALSE(d.command_allowed);
  EXPECT_EQ(d.reason, "cmd_vel timeout, estop active, state communication timeout");
}
