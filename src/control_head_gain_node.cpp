#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>

// ─────────────────────────────────────────────────────────────────────────────
// run_set_gains
//
// CM이 enabled 상태면 disable → 게인 설정 → re-enable 한다.
//
// Parameters:
//   robot_ip      : gRPC 주소 (default: "192.168.3.25:50051")
//   model         : 로봇 모델 ("a" / "m" / "ub", default: "a")
//   head_0_p_gain : head_0 P 게인 (default: 200)
//   head_0_i_gain : head_0 I 게인 (default: 0)
//   head_0_d_gain : head_0 D 게인 (default: 8000)
//   head_1_p_gain : head_1 P 게인 (default: 200)
//   head_1_i_gain : head_1 I 게인 (default: 0)
//   head_1_d_gain : head_1 D 게인 (default: 8000)
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
int run_set_gains(
  const std::string& address,
  uint16_t h0_p, uint16_t h0_i, uint16_t h0_d,
  uint16_t h1_p, uint16_t h1_i, uint16_t h1_d)
{
  using RobotT = rb::Robot<ModelT>;
  auto logger = rclcpp::get_logger("head_gain_node");

  auto robot = RobotT::Create(address);

  RCLCPP_INFO(logger, "Connecting to %s ...", address.c_str());
  if (!robot->Connect()) {
    RCLCPP_FATAL(logger, "Failed to connect to robot at %s", address.c_str());
    return 1;
  }
  RCLCPP_INFO(logger, "Connected");

  // ── CM 상태 확인 → enabled이면 disable 후 Idle 대기 ───────────────────────
  const auto cm_before = robot->GetControlManagerState();
  const bool was_enabled = (cm_before.state == rb::ControlManagerState::State::kEnabled);

  if (was_enabled) {
    RCLCPP_INFO(logger, "CM is enabled, disabling to set head gains ...");
    robot->DisableControlManager();
    for (int i = 0; i < 30; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (robot->GetControlManagerState().state != rb::ControlManagerState::State::kEnabled) break;
      if (i == 29) {
        RCLCPP_FATAL(logger, "CM did not reach Idle after 3s");
        return 1;
      }
    }
    RCLCPP_INFO(logger, "CM disabled");
  }

  // ── 게인 적용 ──────────────────────────────────────────────────────────────
  RCLCPP_INFO(logger, "Applying gains: head_0 P=%-5u I=%-5u D=%u, head_1 P=%-5u I=%-5u D=%u",
    h0_p, h0_i, h0_d, h1_p, h1_i, h1_d);

  if (!robot->SetPositionPIDGain("head_0", h0_p, h0_i, h0_d)) {
    RCLCPP_ERROR(logger, "Failed to set gain for head_0");
    return 1;
  }
  if (!robot->SetPositionPIDGain("head_1", h1_p, h1_i, h1_d)) {
    RCLCPP_ERROR(logger, "Failed to set gain for head_1");
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ── CM re-enable ──────────────────────────────────────────────────────────
  if (was_enabled) {
    if (!robot->EnableControlManager(true /* unlimited_mode */)) {
      RCLCPP_FATAL(logger, "Failed to re-enable control manager");
      return 1;
    }
    RCLCPP_INFO(logger, "Control manager re-enabled");
  }

  RCLCPP_INFO(logger, "Done");
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("control_head_gain_node_param_reader");
  param_node->declare_parameter<std::string>("robot_ip", "192.168.3.25:50051");
  param_node->declare_parameter<std::string>("model",    "a");

  param_node->declare_parameter<int>("head_0_p_gain", 200);
  param_node->declare_parameter<int>("head_0_i_gain", 0);
  param_node->declare_parameter<int>("head_0_d_gain", 8000);
  param_node->declare_parameter<int>("head_1_p_gain", 200);
  param_node->declare_parameter<int>("head_1_i_gain", 0);
  param_node->declare_parameter<int>("head_1_d_gain", 8000);

  const auto address = param_node->get_parameter("robot_ip").as_string();
  const auto model   = param_node->get_parameter("model").as_string();

  const auto h0_p = static_cast<uint16_t>(param_node->get_parameter("head_0_p_gain").as_int());
  const auto h0_i = static_cast<uint16_t>(param_node->get_parameter("head_0_i_gain").as_int());
  const auto h0_d = static_cast<uint16_t>(param_node->get_parameter("head_0_d_gain").as_int());
  const auto h1_p = static_cast<uint16_t>(param_node->get_parameter("head_1_p_gain").as_int());
  const auto h1_i = static_cast<uint16_t>(param_node->get_parameter("head_1_i_gain").as_int());
  const auto h1_d = static_cast<uint16_t>(param_node->get_parameter("head_1_d_gain").as_int());

  int ret = 0;
  if (model == "a") {
    ret = run_set_gains<rb::y1_model::A>(address, h0_p, h0_i, h0_d, h1_p, h1_i, h1_d);
  } else if (model == "m") {
    ret = run_set_gains<rb::y1_model::M>(address, h0_p, h0_i, h0_d, h1_p, h1_i, h1_d);
  } else if (model == "ub") {
    ret = run_set_gains<rb::y1_model::UB>(address, h0_p, h0_i, h0_d, h1_p, h1_i, h1_d);
  } else {
    RCLCPP_FATAL(param_node->get_logger(),
      "Unknown model '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
