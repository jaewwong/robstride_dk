#pragma once

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>

// 🔹 사용 메시지들
#include "robstride_motor_msgs/msg/motor_state.hpp"
#include "robstride_motor_msgs/msg/motor_command.hpp"

// RobStride 드라이버
#include "robstride_dk/robstride_driver.hpp"

namespace robstride_dk {

template <typename C> struct is_vector : std::false_type {};
template <typename T, typename A> struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief RobstrideDkNode class
 */
class RobstrideDkNode : public rclcpp::Node {

 public:
  RobstrideDkNode();
  ~RobstrideDkNode() override;

  /// @brief Number of threads for MultiThreadedExecutor
  int num_threads_ = 1;

 private:

  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
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

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief MotorCommand 들어왔을 때 처리
   */
  void commandCallback(
      const robstride_motor_msgs::msg::MotorCommand::ConstSharedPtr &msg);

  /**
   * @brief 주기적으로 모터 상태를 읽어서 MotorState 퍼블리시
   *
   * 여기서 RobStrideMotor의 CSP 명령을 한 번 보내고,
   * 그 응답에서 받은 position/velocity/torque/temperature를 토픽으로 퍼블리시한다.
   */
  void timerCallback();

 private:
  // ====== 파라미터 관련 ======
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /// @brief CAN 인터페이스 이름 (예: "can0")
  std::string can_interface_ = "can0";

  /// @brief RobStride master ID
  int master_id_ = 1;

  /// @brief 모터 CAN ID (0~255)
  int motor_id_ = 0xFD;

  /// @brief ActuatorType 인덱스 (0~6)
  int actuator_type_ = static_cast<int>(ActuatorType::ROBSTRIDE_01);

  /// @brief 상태 퍼블리시 주기 [Hz]
  double state_pub_rate_ = 100.0;

  /// @brief 노드 시작할 때 자동 enable 할지 여부
  bool auto_enable_ = true;

  /// @brief Dummy parameter (parameter)
  double param_ = 1.0;

  // ====== ROS 통신 객체 ======

  /// @brief Subscriber (MotorCommand)
  rclcpp::Subscription<robstride_motor_msgs::msg::MotorCommand>::SharedPtr command_sub_;

  /// @brief Publisher (MotorState)
  rclcpp::Publisher<robstride_motor_msgs::msg::MotorState>::SharedPtr state_pub_;

  /// @brief Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // ====== 실제 모터 드라이버 ======
  std::shared_ptr<RobStrideMotor> motor_;
};

}  // namespace robstride_dk
