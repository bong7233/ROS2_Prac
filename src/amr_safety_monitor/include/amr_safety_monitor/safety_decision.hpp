// Pure safety-gate decision logic.
//
// Header-only and free of ROS types so the safety-critical "should the command
// be allowed, and why" logic can be unit tested with gtest without a node, and
// so the node has a single source of truth for it. Timing-dependent inputs
// (command timeout, communication fault) are computed by the node and passed in.
#ifndef AMR_SAFETY_MONITOR__SAFETY_DECISION_HPP_
#define AMR_SAFETY_MONITOR__SAFETY_DECISION_HPP_

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace amr_safety_monitor
{

struct SafetyInputs
{
  bool command_timeout;
  bool estop_active;
  bool protective_stop_active;
  bool battery_critical;
  bool motor_fault;
  std::uint32_t motor_fault_code;
  bool require_motor_enabled;
  bool motor_enabled;
  bool communication_fault;
};

struct SafetyDecision
{
  bool command_allowed;
  std::string reason;
};

inline std::string joinReasons(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "clear";
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << reasons.at(i);
  }
  return oss.str();
}

// The command is allowed only when no gating condition is active. The reason
// string lists every active condition in priority order (or "clear").
inline SafetyDecision evaluateSafetyDecision(const SafetyInputs & in)
{
  std::vector<std::string> reasons;
  if (in.command_timeout) {
    reasons.push_back("cmd_vel timeout");
  }
  if (in.estop_active) {
    reasons.push_back("estop active");
  }
  if (in.protective_stop_active) {
    reasons.push_back("protective stop active");
  }
  if (in.battery_critical) {
    reasons.push_back("battery critical");
  }
  if (in.motor_fault) {
    reasons.push_back("motor fault " + std::to_string(in.motor_fault_code));
  }
  if (in.require_motor_enabled && !in.motor_enabled) {
    reasons.push_back("motor disabled");
  }
  if (in.communication_fault) {
    reasons.push_back("state communication timeout");
  }

  SafetyDecision decision;
  decision.command_allowed = reasons.empty();
  decision.reason = joinReasons(reasons);
  return decision;
}

}  // namespace amr_safety_monitor

#endif  // AMR_SAFETY_MONITOR__SAFETY_DECISION_HPP_
