#pragma once

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include <optional>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include "robstride_motor_msgs/msg/motor_state.hpp"
#include "robstride_motor_msgs/msg/motor_command.hpp"
#include "robstride_dk/robstride_driver.hpp"

namespace robstride_dk {

template <typename C> struct is_vector : std::false_type {};
template <typename T, typename A> struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;
// command watchdog
rclcpp::Time last_cmd_time_;
double cmd_timeout_sec_ = 0.3;   // 0.3s 동안 command 없으면 정지
bool motor_disabled_by_timeout_ = false;

class RobstrideDkNode : public rclcpp::Node {
public:
  RobstrideDkNode();
  ~RobstrideDkNode() override;

  int num_threads_ = 1;

private:
  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  void setup();
  void commandCallback(const robstride_motor_msgs::msg::MotorCommand::ConstSharedPtr& msg);
  void timerCallback();

private:
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  std::string can_interface_ = "can0";
  int master_id_ = 1;
  int motor_id_ = 0xFD;
  int actuator_type_ = static_cast<int>(ActuatorType::ROBSTRIDE_01);
  double state_pub_rate_ = 100.0;
  bool auto_enable_ = true;
  double param_ = 1.0;

  rclcpp::Subscription<robstride_motor_msgs::msg::MotorCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<robstride_motor_msgs::msg::MotorState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<RobStrideMotor> motor_;
};

// ✅ 템플릿 구현은 헤더에 있어야 링크에러 안남
template <typename T>
void RobstrideDkNode::declareAndLoadParameter(
    const std::string &name,
    T &param,
    const std::string &description,
    const bool add_to_auto_reconfigurable_params,
    const bool is_required,
    const bool read_only,
    const std::optional<double> &from_value,
    const std::optional<double> &to_value,
    const std::optional<double> &step_value,
    const std::string &additional_constraints)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  desc.read_only = read_only;
  desc.additional_constraints = additional_constraints;

  auto type = rclcpp::ParameterValue(param).get_type();
  this->declare_parameter(name, type, desc);

  try {
    param = this->get_parameter(name).get_value<T>();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    if (is_required) {
      RCLCPP_FATAL(this->get_logger(), "Missing required parameter '%s'", name.c_str());
      rclcpp::shutdown();
      std::exit(1);
    }
    this->set_parameter(rclcpp::Parameter(name, param));
  }

  if (add_to_auto_reconfigurable_params) {
    auto setter = [&param](const rclcpp::Parameter &p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.emplace_back(name, setter);
  }
}

} // namespace robstride_dk
