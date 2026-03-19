#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>

// ─────────────────────────────────────────────────────────────────────────────
// run_turn_off
//
// 상태 확인 후 순서대로 종료:
//   control manager off → servo off → power off
// 각 단계에서 이미 off 상태이면 스킵한다 (best-effort).
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
int run_turn_off(const std::string& address,
                 const std::string& servo,
                 const std::string& power) {
  using RobotT = rb::Robot<ModelT>;

  auto logger = rclcpp::get_logger("turn_off_node");

  auto robot = RobotT::Create(address);

  RCLCPP_INFO(logger, "Connecting to robot at %s ...", address.c_str());
  if (!robot->Connect()) {
    RCLCPP_FATAL(logger, "Failed to connect to robot");
    return 1;
  }
  RCLCPP_INFO(logger, "Connected");

  int exit_code = 0;

  // ── Step 1: Control Manager 비활성화 ─────────────────────────────────────────
  {
    const auto& cm = robot->GetControlManagerState();
    if (cm.state == rb::ControlManagerState::State::kEnabled) {
      RCLCPP_INFO(logger, "Disabling control manager ...");
      if (!robot->DisableControlManager()) {
        RCLCPP_ERROR(logger, "Failed to disable control manager (continuing...)");
        exit_code = 1;
      } else {
        RCLCPP_INFO(logger, "Control manager disabled");
      }
    } else {
      RCLCPP_INFO(logger, "Control manager already inactive (state=%s), skipping",
                  rb::to_string(cm.state).c_str());
    }
  }

  // ── Step 2: Servo Off ─────────────────────────────────────────────────────────
  {
    if (robot->IsServoOn(servo)) {
      RCLCPP_INFO(logger, "Turning servo off ('%s') ...", servo.c_str());
      if (!robot->ServoOff(servo)) {
        RCLCPP_ERROR(logger, "Failed to turn servo off (continuing...)");
        exit_code = 1;
      } else {
        RCLCPP_INFO(logger, "Servo off");
      }
    } else {
      RCLCPP_INFO(logger, "Servo already off, skipping");
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // ── Step 3: Power Off ─────────────────────────────────────────────────────────
  {
    if (robot->IsPowerOn(power)) {
      RCLCPP_INFO(logger, "Powering off ('%s') ...", power.c_str());
      if (!robot->PowerOff(power)) {
        RCLCPP_ERROR(logger, "Failed to power off (continuing...)");
        exit_code = 1;
      } else {
        RCLCPP_INFO(logger, "Power off");
      }
    } else {
      RCLCPP_INFO(logger, "Power already off, skipping");
    }
  }

  RCLCPP_INFO(logger, "Shutdown sequence complete");
  return exit_code;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("turn_off_param_reader");
  param_node->declare_parameter<std::string>("robot_ip", "192.168.3.25:50051");
  param_node->declare_parameter<std::string>("model", "a");
  param_node->declare_parameter<std::string>("servo", ".*");
  param_node->declare_parameter<std::string>("power", ".*");

  const auto address = param_node->get_parameter("robot_ip").as_string();
  const auto model   = param_node->get_parameter("model").as_string();
  const auto servo   = param_node->get_parameter("servo").as_string();
  const auto power   = param_node->get_parameter("power").as_string();

  int ret = 0;
  if (model == "a") {
    ret = run_turn_off<rb::y1_model::A>(address, servo, power);
  } else if (model == "m") {
    ret = run_turn_off<rb::y1_model::M>(address, servo, power);
  } else if (model == "ub") {
    ret = run_turn_off<rb::y1_model::UB>(address, servo, power);
  } else {
    RCLCPP_FATAL(param_node->get_logger(),
                 "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
