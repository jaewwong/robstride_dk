#include <chrono>
#include <thread>
#include <functional>

#include "robstride_dk/robstride_dk_node.hpp"

using namespace std::chrono_literals;

namespace robstride_dk {

RobstrideDkNode::RobstrideDkNode() : Node("robstride_dk_node") {

  this->declareAndLoadParameter(
      "num_threads", num_threads_,
      "number of threads for MultiThreadedExecutor",
      false, false, false,
      1, std::thread::hardware_concurrency(), 1);

  this->declareAndLoadParameter("can_interface", can_interface_, "CAN interface name (e.g. can0)");
  this->declareAndLoadParameter("master_id", master_id_, "RobStride master ID", true, false, false, 0, 255, 1);
  this->declareAndLoadParameter("motor_id", motor_id_, "Motor CAN ID", true, false, false, 0, 255, 1);
  this->declareAndLoadParameter("actuator_type", actuator_type_, "ActuatorType index (0~6)", true, false, false, 0, 6, 1);
  this->declareAndLoadParameter("state_pub_rate", state_pub_rate_, "MotorState publish rate [Hz]", true, false, false, 1.0, 1000.0, 1.0);
  this->declareAndLoadParameter("auto_enable", auto_enable_, "Automatically enable motor at startup");
  this->declareAndLoadParameter("param", param_, "Dummy param (kept from template)", true, false, false, 0.0, 10.0, 1.0);

  try {
    motor_ = std::make_shared<RobStrideMotor>(
        can_interface_,
        static_cast<uint8_t>(master_id_),
        static_cast<uint8_t>(motor_id_),
        actuator_type_);

    RCLCPP_INFO(
        this->get_logger(),
        "RobStrideMotor created (iface=%s, master_id=%d, motor_id=0x%02X, actuator_type=%d)",
        can_interface_.c_str(), master_id_, motor_id_, actuator_type_);

    if (auto_enable_) {
      RCLCPP_INFO(this->get_logger(), "Auto-enabling motor...");
      motor_->enable_motor();
    }
  } catch (const std::exception &e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create RobStrideMotor: %s", e.what());
    throw;
  }

  last_cmd_time_ = this->now();
motor_disabled_by_timeout_ = false;


  this->setup();
}

RobstrideDkNode::~RobstrideDkNode() {
  RCLCPP_INFO(this->get_logger(), "Shutting down RobstrideDkNode...");
  if (motor_) {
    try { motor_->Disenable_Motor(0); } catch (...) {}
  }
}

rcl_interfaces::msg::SetParametersResult
RobstrideDkNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  for (const auto& p : parameters) {
    for (auto& item : auto_reconfigurable_params_) {
      if (p.get_name() == std::get<0>(item)) {
        std::get<1>(item)(p);
      }
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}



void RobstrideDkNode::setup() {
  parameters_callback_ =
      this->add_on_set_parameters_callback(
          std::bind(&RobstrideDkNode::parametersCallback, this, std::placeholders::_1));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  command_sub_ =
      this->create_subscription<robstride_motor_msgs::msg::MotorCommand>(
          "~/command",
          10,
          std::bind(&RobstrideDkNode::commandCallback, this, std::placeholders::_1),
          sub_options);

  state_pub_ =
      this->create_publisher<robstride_motor_msgs::msg::MotorState>("~/state", 10);

  auto period = std::chrono::duration<double>(1.0 / state_pub_rate_);
  timer_ = this->create_wall_timer(period, std::bind(&RobstrideDkNode::timerCallback, this));
}

void RobstrideDkNode::commandCallback(
    const robstride_motor_msgs::msg::MotorCommand::ConstSharedPtr& msg)
{
  last_cmd_time_ = this->now();
  motor_disabled_by_timeout_ = false;

  if (!motor_) return;

  try {
    if (msg->enable == 0) { motor_->Disenable_Motor(0); return; }
    if (msg->enable == 1) { motor_->enable_motor(); }

    switch (msg->mode) {
      case robstride_motor_msgs::msg::MotorCommand::MOTION: {
        const auto & m = msg->motion;
        motor_->send_motion_command(m.torque, m.position, m.velocity, m.kp, m.kd);
        break;
      }
      case robstride_motor_msgs::msg::MotorCommand::PP: {
        const auto & p = msg->pp;
        motor_->RobStrite_Motor_PosPP_control(p.limit_speed, p.acceleration, p.position);
        break;
      }
      case robstride_motor_msgs::msg::MotorCommand::SPEED: {
        const auto & s = msg->speed;
        motor_->send_velocity_mode_command(s.velocity);
        motor_->Set_RobStrite_Motor_parameter(0x7026, 5.0f, 'p');
        motor_->Set_RobStrite_Motor_parameter(0x7018, 10.0f, 'p');
        break;
      }
      case robstride_motor_msgs::msg::MotorCommand::CURRENT: {
        const auto & c = msg->current;
        motor_->RobStrite_Motor_Current_control(c.iq, c.id);
        break;
      }
      case robstride_motor_msgs::msg::MotorCommand::CSP: {
        const auto & c = msg->csp;
        motor_->RobStrite_Motor_PosCSP_control(c.velocity, c.position);
        break;
      }
      case robstride_motor_msgs::msg::MotorCommand::ZERO: {
        motor_->Set_ZeroPos();
        break;
      }
      default:
        break;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "commandCallback exception: %s", e.what());
  }
}

void RobstrideDkNode::timerCallback() {
  if (!motor_) return;

  // =========================
  // 🔥 command watchdog
  // =========================
  const double dt = (this->now() - last_cmd_time_).seconds();
  if (dt > cmd_timeout_sec_) {
    if (!motor_disabled_by_timeout_) {
      RCLCPP_WARN(
          this->get_logger(),
          "No command for %.3f sec -> DISABLE motor", dt);
      try {
        motor_->Disenable_Motor(0);   // ✅ 모터 정지
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Disable failed: %s", e.what());
      }
      motor_disabled_by_timeout_ = true;
    }
  }

  // =========================
  // (선택) 상태 퍼블리시
  // =========================
  // 👉 퍼블리시를 아예 끄고 싶으면 이 아래 블록 통째로 주석 처리해도 됨
  robstride_motor_msgs::msg::MotorState state_msg;
  state_msg.id          = static_cast<uint8_t>(motor_id_);
  state_msg.position    = motor_->position_;
  state_msg.velocity    = motor_->velocity_;
  state_msg.torque      = motor_->torque_;
  state_msg.temperature = motor_->temperature_;
  state_msg.mode        = static_cast<uint8_t>(motor_->drw.run_mode.data);
  state_msg.error_code  = motor_->error_code;
  state_msg.pattern     = motor_->pattern;

  state_pub_->publish(state_msg);
}

} // namespace robstride_dk

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robstride_dk::RobstrideDkNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), node->num_threads_);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
