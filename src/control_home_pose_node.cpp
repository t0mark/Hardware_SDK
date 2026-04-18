#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <rby1-sdk/robot_command_builder.h>

// ─────────────────────────────────────────────────────────────────────────────
// run_home_pose
//
// 이미 실행 중인 hardware_node와 공존하여 home pose 커맨드를 전송하고 종료한다.
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
int run_home_pose(const std::string& address, double minimum_time) {
  using RobotT = rb::Robot<ModelT>;

  auto logger = rclcpp::get_logger("home_pose_node");

  auto robot = RobotT::Create(address);

  RCLCPP_INFO(logger, "Connecting to robot at %s ...", address.c_str());
  if (!robot->Connect()) {
    RCLCPP_FATAL(logger, "Failed to connect to robot");
    return 1;
  }
  RCLCPP_INFO(logger, "Connected");

  // 상태 업데이트 시작 (connection 안정화용)
  robot->StartStateUpdate([](const auto&) {}, 10.0 /* Hz */);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // ── Control manager 준비 대기 (turn_on_hardware_node가 enable할 때까지 폴링) ──
  constexpr int kMaxRetries = 30;
  for (int i = 0; i < kMaxRetries; ++i) {
    const auto& cm = robot->GetControlManagerState();
    if (cm.state == rb::ControlManagerState::State::kEnabled) break;
    if (i == kMaxRetries - 1) {
      RCLCPP_FATAL(logger, "Control manager not enabled after %d retries", kMaxRetries);
      return 1;
    }
    RCLCPP_INFO(logger, "Waiting for control manager to be enabled (%d/%d) ...", i + 1, kMaxRetries);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO(logger, "Control manager ready");

  // ── Home pose 커맨드 빌드 ────────────────────────────────────────────────────
  constexpr size_t torso_dof     = ModelT::kTorsoIdx.size();
  constexpr size_t right_arm_dof = ModelT::kRightArmIdx.size();
  constexpr size_t left_arm_dof  = ModelT::kLeftArmIdx.size();
  constexpr size_t head_dof      = ModelT::kHeadIdx.size();

  // torso: [0, π/4, -π/2, π/4, 0, 0]
  Eigen::VectorXd q_torso(torso_dof);
  q_torso << 0.0, 0.7854, -1.5708, 0.7854, 0.0, 0.0;

  // right arm: [0, -0.0873, 0, -2.0944, 0, 1.2217, 0]
  Eigen::VectorXd q_right_arm(right_arm_dof);
  q_right_arm << 0.0, -0.0873, 0.0, -2.0944, 0.0, 1.2217, 0.0;

  // left arm: [0, 0.0873, 0, -2.0944, 0, 1.2217, 0]
  Eigen::VectorXd q_left_arm(left_arm_dof);
  q_left_arm << 0.0, 0.0873, 0.0, -2.0944, 0.0, 1.2217, 0.0;

  // head: [0, 0]
  Eigen::VectorXd q_head = Eigen::VectorXd::Zero(head_dof);

  RCLCPP_INFO(logger, "Moving to home pose (minimum_time=%.1fs) ...", minimum_time);

  auto rv = robot->SendCommand(
    rb::RobotCommandBuilder().SetCommand(
      rb::ComponentBasedCommandBuilder()
        .SetBodyCommand(
          rb::BodyComponentBasedCommandBuilder()
            .SetTorsoCommand(
              rb::JointPositionCommandBuilder()
                .SetMinimumTime(minimum_time)
                .SetPosition(q_torso))
            .SetRightArmCommand(
              rb::JointPositionCommandBuilder()
                .SetMinimumTime(minimum_time)
                .SetPosition(q_right_arm))
            .SetLeftArmCommand(
              rb::JointPositionCommandBuilder()
                .SetMinimumTime(minimum_time)
                .SetPosition(q_left_arm)))
        .SetHeadCommand(
          rb::HeadCommandBuilder().SetCommand(
            rb::JointPositionCommandBuilder()
              .SetMinimumTime(minimum_time)
              .SetPosition(q_head)))
    )
  )->Get();

  if (rv.finish_code() != rb::RobotCommandFeedback::FinishCode::kOk) {
    RCLCPP_ERROR(logger, "Home pose command failed (finish_code=%d)",
                 static_cast<int>(rv.finish_code()));
    return 1;
  }

  RCLCPP_INFO(logger, "Home pose reached successfully");
  robot->StopStateUpdate();
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("control_home_pose_node_param_reader");
  param_node->declare_parameter<std::string>("robot_ip", "192.168.3.25:50051");
  param_node->declare_parameter<std::string>("model", "a");
  param_node->declare_parameter<double>("minimum_time", 10.0);

  const auto address      = param_node->get_parameter("robot_ip").as_string();
  const auto model        = param_node->get_parameter("model").as_string();
  const auto minimum_time = param_node->get_parameter("minimum_time").as_double();

  int ret = 0;
  if (model == "a") {
    ret = run_home_pose<rb::y1_model::A>(address, minimum_time);
  } else if (model == "m") {
    ret = run_home_pose<rb::y1_model::M>(address, minimum_time);
  } else if (model == "ub") {
    ret = run_home_pose<rb::y1_model::UB>(address, minimum_time);
  } else {
    RCLCPP_FATAL(param_node->get_logger(),
                 "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
