#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>

// ─────────────────────────────────────────────────────────────────────────────
// run_set_gains
//
// 파라미터로 지정된 PID 게인을 헤드 조인트에 적용하고 종료한다.
//
// Parameters:
//   robot_ip      : gRPC 주소 (default: "192.168.3.25:50051")
//   model         : 로봇 모델 ("a" / "m" / "ub", default: "a")
//   head_0_p_gain : head_0 P 게인 (default: 800)
//   head_0_i_gain : head_0 I 게인 (default: 0)
//   head_0_d_gain : head_0 D 게인 (default: 4000)
//   head_1_p_gain : head_1 P 게인 (default: 800)
//   head_1_i_gain : head_1 I 게인 (default: 0)
//   head_1_d_gain : head_1 D 게인 (default: 4000)
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

  // ── 현재 게인 확인 (Control Manager 활성 시 읽기 불가 → 스킵) ───────────────
  try {
    const auto before = robot->GetHeadPositionPIDGains();
    RCLCPP_INFO(logger, "Current head gains:");
    for (size_t i = 0; i < before.size(); ++i) {
      RCLCPP_INFO(logger, "  head_%zu  P=%-5u  I=%-5u  D=%u",
        i, before[i].p_gain, before[i].i_gain, before[i].d_gain);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger, "Could not read current gains (skipping): %s", e.what());
  }

  // ── 게인 적용 ──────────────────────────────────────────────────────────────
  RCLCPP_INFO(logger, "Applying new gains:");
  RCLCPP_INFO(logger, "  head_0  P=%-5u  I=%-5u  D=%u", h0_p, h0_i, h0_d);
  RCLCPP_INFO(logger, "  head_1  P=%-5u  I=%-5u  D=%u", h1_p, h1_i, h1_d);

  if (!robot->SetPositionPIDGain("head_0", h0_p, h0_i, h0_d)) {
    RCLCPP_ERROR(logger, "Failed to set gain for head_0");
    return 1;
  }
  if (!robot->SetPositionPIDGain("head_1", h1_p, h1_i, h1_d)) {
    RCLCPP_ERROR(logger, "Failed to set gain for head_1");
    return 1;
  }

  // ── 적용 후 확인 (Control Manager 활성 시 읽기 불가 → 스킵) ─────────────────
  try {
    const auto after = robot->GetHeadPositionPIDGains();
    RCLCPP_INFO(logger, "Verified head gains:");
    for (size_t i = 0; i < after.size(); ++i) {
      RCLCPP_INFO(logger, "  head_%zu  P=%-5u  I=%-5u  D=%u",
        i, after[i].p_gain, after[i].i_gain, after[i].d_gain);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger, "Could not verify gains (skipping): %s", e.what());
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

  auto param_node = rclcpp::Node::make_shared("head_gain_param_reader");
  param_node->declare_parameter<std::string>("robot_ip", "192.168.3.25:50051");
  param_node->declare_parameter<std::string>("model",    "a");

  param_node->declare_parameter<int>("head_0_p_gain", 400);
  param_node->declare_parameter<int>("head_0_i_gain", 0);
  param_node->declare_parameter<int>("head_0_d_gain", 0);
  param_node->declare_parameter<int>("head_1_p_gain", 400);
  param_node->declare_parameter<int>("head_1_i_gain", 0);
  param_node->declare_parameter<int>("head_1_d_gain", 0);

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
