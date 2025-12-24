#include <chrono>
#include <functional>
#include <thread>
#include <sstream>
#include <atomic>

#include "robstride_dk/robstride_dk_node.hpp"
#include "robstride_dk/robstride_driver.hpp"

#include "robstride_motor_msgs/msg/motor_command.hpp"
#include "robstride_motor_msgs/msg/motor_state.hpp"

using namespace std::chrono_literals;

namespace robstride_dk {

RobstrideDkNode::RobstrideDkNode() : Node("robstride_dk_node") {

  // ---- 파라미터 선언 & 로드 ----
  this->declareAndLoadParameter(
      "num_threads",
      num_threads_,
      "number of threads for MultiThreadedExecutor",
      false, false, false,
      1, std::thread::hardware_concurrency(), 1);

  this->declareAndLoadParameter(
      "can_interface",
      can_interface_,
      "CAN interface name (e.g. can0)",
      true, false, false);

  this->declareAndLoadParameter(
      "master_id",
      master_id_,
      "RobStride master ID",
      true, false, false,
      0, 255, 1);

  this->declareAndLoadParameter(
      "motor_id",
      motor_id_,
      "Motor CAN ID",
      true, false, false,
      0, 255, 1);

  this->declareAndLoadParameter(
      "actuator_type",
      actuator_type_,
      "ActuatorType index (0~6)",
      true, false, false,
      0, 6, 1);

  this->declareAndLoadParameter(
      "state_pub_rate",
      state_pub_rate_,
      "MotorState publish rate [Hz]",
      true, false, false,
      1.0, 1000.0, 1.0);

  this->declareAndLoadParameter(
      "auto_enable",
      auto_enable_,
      "Automatically enable motor at startup",
      true, false, false);

  this->declareAndLoadParameter(
      "param",
      param_,
      "Dummy param (kept from template)",
      true, false, false,
      0.0, 10.0, 1.0);

  // ---- 드라이버 생성 ----
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
  }
  catch (const std::exception &e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create RobStrideMotor: %s", e.what());
    throw;
  }

  // ⬆️ 여기까지는 드라이버 초기화 + enable까지만 하고,
  // ⬇️ 실제 상태 갱신은 timerCallback에서 CSP 명령을 보내면서 응답을 읽는 폴링 방식으로 처리
  this->setup();
}

// 🔹 소멸자: 모터 디세이블
RobstrideDkNode::~RobstrideDkNode() {
  RCLCPP_INFO(this->get_logger(), "Shutting down RobstrideDkNode...");

  if (motor_) {
    try {
      motor_->Disenable_Motor(0);
    }
    catch (...) {
      // 무시
    }
  }

  RCLCPP_INFO(this->get_logger(), "RobstrideDkNode destroyed.");
}

// ========= 템플릿 파라미터 함수 구현 =========

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
    const std::string &additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<int64_t>(from_value.value()))
           .set__to_value(static_cast<int64_t>(to_value.value()));
      if (step_value.has_value()) {
        range.set__step(static_cast<int64_t>(step_value.value()));
      }
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<double>(from_value.value()))
           .set__to_value(static_cast<double>(to_value.value()));
      if (step_value.has_value()) {
        range.set__step(static_cast<double>(step_value.value()));
      }
      param_desc.floating_point_range = {range};
    } else {
      // string, bool 등은 range 필요 없음
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto &element : param) {
        ss << element << (&element != &param.back() ? ", " : "");
      }
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  }
  catch (rclcpp::exceptions::ParameterUninitializedException &) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto &element : param) {
          ss << element << (&element != &param.back() ? ", " : "");
        }
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter &)> setter =
        [&param](const rclcpp::Parameter &p) {
          param = p.get_value<T>();
        };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

// 템플릿 명시적 인스턴스
template void RobstrideDkNode::declareAndLoadParameter<int>(
    const std::string &, int &, const std::string &, const bool,
    const bool, const bool, const std::optional<double> &,
    const std::optional<double> &, const std::optional<double> &,
    const std::string &);

template void RobstrideDkNode::declareAndLoadParameter<double>(
    const std::string &, double &, const std::string &, const bool,
    const bool, const bool, const std::optional<double> &,
    const std::optional<double> &, const std::optional<double> &,
    const std::string &);

template void RobstrideDkNode::declareAndLoadParameter<bool>(
    const std::string &, bool &, const std::string &, const bool,
    const bool, const bool, const std::optional<double> &,
    const std::optional<double> &, const std::optional<double> &,
    const std::string &);

template void RobstrideDkNode::declareAndLoadParameter<std::string>(
    const std::string &, std::string &, const std::string &, const bool,
    const bool, const bool, const std::optional<double> &,
    const std::optional<double> &, const std::optional<double> &,
    const std::string &);

// ========= 파라미터 콜백 =========

rcl_interfaces::msg::SetParametersResult RobstrideDkNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {

  for (const auto &param : parameters) {
    for (auto &auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(
            this->get_logger(),
            "Reconfigured parameter '%s' to: %s",
            param.get_name().c_str(),
            param.value_to_string().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// ========= setup() =========

void RobstrideDkNode::setup() {

  // dynamic parameter callback
  parameters_callback_ =
      this->add_on_set_parameters_callback(
          std::bind(&RobstrideDkNode::parametersCallback, this, std::placeholders::_1));

  // callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 🔹 MotorCommand subscriber
  command_sub_ =
      this->create_subscription<robstride_motor_msgs::msg::MotorCommand>(
          "~/command",   // 예: /robstride_dk_node/command
          10,
          std::bind(&RobstrideDkNode::commandCallback, this, std::placeholders::_1),
          sub_options);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", command_sub_->get_topic_name());

  // 🔹 MotorState publisher
  state_pub_ =
      this->create_publisher<robstride_motor_msgs::msg::MotorState>("~/state", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", state_pub_->get_topic_name());

  // 🔹 Timer: 모터 상태 publish + CSP 폴링
  auto period = std::chrono::duration<double>(1.0 / state_pub_rate_);
  timer_ = this->create_wall_timer(
      period,
      std::bind(&RobstrideDkNode::timerCallback, this));
  RCLCPP_INFO(this->get_logger(), "State publish rate: %.1f Hz", state_pub_rate_);
}

// ========= 명령 콜백 =========

void RobstrideDkNode::commandCallback(
    const robstride_motor_msgs::msg::MotorCommand::ConstSharedPtr &msg) {

  if (!motor_) {
    RCLCPP_ERROR(this->get_logger(), "Motor driver not initialized");
    return;
  }

  // ID 체크(옵션)
  if (msg->id != static_cast<uint8_t>(motor_id_)) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Received command for id=%d, but this node motor_id=%d, ignoring",
        msg->id, motor_id_);
    // 필요하면 여기서 그냥 return; 해도 된다
  }

  try {
    // enable / disable
    if (msg->enable == 0) {
      RCLCPP_INFO(this->get_logger(), "Disable motor (clear_error=0)");
      motor_->Disenable_Motor(0);
      return;
    } else if (msg->enable == 1) {
      RCLCPP_INFO(this->get_logger(), "Enable motor");
      motor_->enable_motor();
    }

    // 모드별로 분기
    switch (msg->mode) {
      case robstride_motor_msgs::msg::MotorCommand::MOTION: {
        const auto &m = msg->motion;
        RCLCPP_INFO(
            this->get_logger(),
            "MOTION cmd: torque=%.3f, pos=%.3f, vel=%.3f, kp=%.3f, kd=%.3f",
            m.torque, m.position, m.velocity, m.kp, m.kd);
        motor_->send_motion_command(
            static_cast<float>(m.torque),
            static_cast<float>(m.position),
            static_cast<float>(m.velocity),
            static_cast<float>(m.kp),
            static_cast<float>(m.kd));
        break;
      }

      case robstride_motor_msgs::msg::MotorCommand::PP: {
        // MotorPP.msg: position, acceleration, limit_speed
        const auto &p = msg->pp;
        RCLCPP_INFO(
            this->get_logger(),
            "PP cmd: limit_speed=%.3f rad/s, acc=%.3f rad/s^2, pos=%.3f rad",
            p.limit_speed, p.acceleration, p.position);
        motor_->RobStrite_Motor_PosPP_control(
            static_cast<float>(p.limit_speed),
            static_cast<float>(p.acceleration),
            static_cast<float>(p.position));
        break;
      }

      case robstride_motor_msgs::msg::MotorCommand::SPEED: {
        // MotorSpeed.msg: velocity
        const auto &s = msg->speed;
        RCLCPP_INFO(
            this->get_logger(),
            "SPEED cmd: velocity=%.3f rad/s", s.velocity);
        motor_->send_velocity_mode_command(
            static_cast<float>(s.velocity));
        break;
      }

      case robstride_motor_msgs::msg::MotorCommand::CURRENT: {
        const auto &c = msg->current;
        RCLCPP_INFO(
            this->get_logger(),
            "CURRENT cmd: iq=%.3f, id=%.3f",
            c.iq, c.id);
        motor_->RobStrite_Motor_Current_control(
            static_cast<float>(c.iq),
            static_cast<float>(c.id));
        break;
      }

      case robstride_motor_msgs::msg::MotorCommand::CSP: {
        // MotorCSP.msg: position, velocity
        const auto &c = msg->csp;
        RCLCPP_INFO(
            this->get_logger(),
            "CSP cmd: velocity=%.3f rad/s, position=%.3f rad",
            c.velocity, c.position);
        motor_->RobStrite_Motor_PosCSP_control(
            static_cast<float>(c.velocity),
            static_cast<float>(c.position));
        break;
      }

      case robstride_motor_msgs::msg::MotorCommand::ZERO: {
        RCLCPP_INFO(this->get_logger(), "Set zero position");
        motor_->Set_ZeroPos();
        break;
      }

      default:
        RCLCPP_WARN(this->get_logger(), "Unknown mode: %d", msg->mode);
        break;
    }
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in commandCallback: %s", e.what());
  }
}

// ========= 타이머 콜백 =========
//
// 여기서 샘플 코드처럼 "명령 1번 보내고 → 그 응답에서 상태 읽기" 방식으로 폴링.
// 현재 위치 유지(CSP, speed=0, angle=현재각)로 주기적으로 명령을 보내면서
// 모터가 회신하는 MotorRequest(0x02) 프레임을 이용해 상태를 갱신한다.
void RobstrideDkNode::timerCallback() {

  if (!motor_) return;

  try {
    // 1) 현재 위치 유지하도록 CSP 명령 전송
    //    - Speed = 0.0f → 정지 명령
    //    - Angle = motor_->position_ → 현재 위치 유지
    auto [pos, vel, tq, temp] =
        motor_->RobStrite_Motor_PosCSP_control(
            0.0f,
            motor_->position_);

    // 2) 받은 값을 MotorState로 퍼블리시
    robstride_motor_msgs::msg::MotorState state_msg;
    state_msg.id          = static_cast<uint8_t>(motor_id_);
    state_msg.position    = pos;
    state_msg.velocity    = vel;
    state_msg.torque      = tq;
    state_msg.temperature = temp;
    state_msg.mode        = static_cast<uint8_t>(motor_->drw.run_mode.data);
    state_msg.error_code  = motor_->error_code;
    state_msg.pattern     = motor_->pattern;

    state_pub_->publish(state_msg);
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(),
                "Exception in timerCallback: %s", e.what());
  }
}

}  // namespace robstride_dk

// ========= main =========

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<robstride_dk::RobstrideDkNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), node->num_threads_);
  RCLCPP_INFO(
      node->get_logger(),
      "Spinning node '%s' with %s (%d threads)",
      node->get_fully_qualified_name(),
      "MultiThreadedExecutor",
      node->num_threads_);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
