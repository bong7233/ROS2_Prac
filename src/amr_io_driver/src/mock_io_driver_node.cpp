#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_interfaces/msg/io_state.hpp"
#include "amr_interfaces/srv/set_digital_input.hpp"
#include "amr_interfaces/srv/set_digital_output.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_io_driver
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

std::vector<std::string> makeNames(const std::string & prefix, const std::size_t count)
{
  std::vector<std::string> names;
  names.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    names.push_back(prefix + std::to_string(i));
  }
  return names;
}
}  // namespace

class MockIoDriverNode final : public rclcpp::Node
{
public:
  MockIoDriverNode()
  : Node("mock_io_driver"),
    updater_(this)
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    input_count_ = static_cast<std::size_t>(declare_parameter<int>("input_count", 8));
    output_count_ = static_cast<std::size_t>(declare_parameter<int>("output_count", 8));
    estop_input_index_ = declare_parameter<int>("estop_input_index", 0);
    protective_stop_input_index_ = declare_parameter<int>("protective_stop_input_index", 1);
    initial_estop_active_ = declare_parameter<bool>("initial_estop_active", false);
    initial_protective_stop_active_ =
      declare_parameter<bool>("initial_protective_stop_active", false);
    hardware_id_ = declare_parameter<std::string>("hardware_id", "mock_io_board");

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (input_count_ == 0U || output_count_ == 0U) {
      throw std::runtime_error("input_count and output_count must be greater than zero");
    }

    input_names_ = makeNames("di", input_count_);
    output_names_ = makeNames("do", output_count_);
    inputs_.assign(input_count_, false);
    outputs_.assign(output_count_, false);
    setInput(estop_input_index_, initial_estop_active_);
    setInput(protective_stop_input_index_, initial_protective_stop_active_);

    io_pub_ = create_publisher<amr_interfaces::msg::IoState>("io_state", rclcpp::QoS(10));
    set_io_srv_ = create_service<amr_interfaces::srv::SetDigitalOutput>(
      "set_io",
      std::bind(
        &MockIoDriverNode::onSetDigitalOutput,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    set_input_srv_ = create_service<amr_interfaces::srv::SetDigitalInput>(
      "set_input",
      std::bind(
        &MockIoDriverNode::onSetDigitalInput,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    updater_.setHardwareID(hardware_id_);
    updater_.add("io_board", this, &MockIoDriverNode::produceDiagnostics);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&MockIoDriverNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Mock IO driver started: %zu inputs, %zu outputs",
      input_count_,
      output_count_);
  }

private:
  void setInput(const int index, const bool value)
  {
    if (index < 0 || static_cast<std::size_t>(index) >= inputs_.size()) {
      return;
    }
    inputs_.at(static_cast<std::size_t>(index)) = value;
  }

  bool inputActive(const int index) const
  {
    if (index < 0 || static_cast<std::size_t>(index) >= inputs_.size()) {
      return false;
    }
    return inputs_.at(static_cast<std::size_t>(index));
  }

  void onSetDigitalOutput(
    const std::shared_ptr<amr_interfaces::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<amr_interfaces::srv::SetDigitalOutput::Response> response)
  {
    if (request->channel >= outputs_.size()) {
      response->success = false;
      response->message = "output channel is out of range";
      RCLCPP_WARN(
        get_logger(),
        "Rejected set_io request for invalid channel %u",
        request->channel);
      return;
    }

    outputs_.at(request->channel) = request->value;
    response->success = true;
    std::ostringstream oss;
    oss << output_names_.at(request->channel) << " set to "
        << (request->value ? "ON" : "OFF");
    response->message = oss.str();

    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void onSetDigitalInput(
    const std::shared_ptr<amr_interfaces::srv::SetDigitalInput::Request> request,
    std::shared_ptr<amr_interfaces::srv::SetDigitalInput::Response> response)
  {
    if (request->channel >= inputs_.size()) {
      response->success = false;
      response->message = "input channel is out of range";
      RCLCPP_WARN(
        get_logger(),
        "Rejected set_input request for invalid channel %u",
        request->channel);
      return;
    }

    inputs_.at(request->channel) = request->value;
    response->success = true;
    std::ostringstream oss;
    oss << input_names_.at(request->channel) << " forced to "
        << (request->value ? "ON" : "OFF");
    response->message = oss.str();

    RCLCPP_WARN(get_logger(), "FAE scenario: %s", response->message.c_str());
  }

  void onTimer()
  {
    auto msg = amr_interfaces::msg::IoState();
    msg.header.stamp = now();
    msg.header.frame_id = "io_board";
    msg.communication_ok = true;
    msg.estop_active = inputActive(estop_input_index_);
    msg.protective_stop_active = inputActive(protective_stop_input_index_);
    msg.input_names = input_names_;
    msg.inputs = inputs_;
    msg.output_names = output_names_;
    msg.outputs = outputs_;
    msg.sequence = sequence_++;

    io_pub_->publish(msg);
    updater_.force_update();
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const bool estop_active = inputActive(estop_input_index_);
    const bool protective_stop_active = inputActive(protective_stop_input_index_);

    if (estop_active) {
      status.summary(DiagnosticStatus::ERROR, "estop input is active");
    } else if (protective_stop_active) {
      status.summary(DiagnosticStatus::WARN, "protective stop input is active");
    } else {
      status.summary(DiagnosticStatus::OK, "io board is healthy");
    }

    status.add("driver_type", "mock");
    status.add("input_count", static_cast<int>(input_count_));
    status.add("output_count", static_cast<int>(output_count_));
    status.add("estop_input_index", estop_input_index_);
    status.add("protective_stop_input_index", protective_stop_input_index_);
    status.add("estop_active", estop_active);
    status.add("protective_stop_active", protective_stop_active);
    status.add("sequence", sequence_);
  }

  std::string hardware_id_;
  double publish_rate_hz_{10.0};
  std::size_t input_count_{8U};
  std::size_t output_count_{8U};
  int estop_input_index_{0};
  int protective_stop_input_index_{1};
  bool initial_estop_active_{false};
  bool initial_protective_stop_active_{false};
  uint32_t sequence_{0U};

  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<bool> inputs_;
  std::vector<bool> outputs_;

  rclcpp::Publisher<amr_interfaces::msg::IoState>::SharedPtr io_pub_;
  rclcpp::Service<amr_interfaces::srv::SetDigitalOutput>::SharedPtr set_io_srv_;
  rclcpp::Service<amr_interfaces::srv::SetDigitalInput>::SharedPtr set_input_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_io_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_io_driver::MockIoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
