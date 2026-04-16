#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <rby1-sdk/robot_command_builder.h>

// ─────────────────────────────────────────────────────────────────────────────
// RBY1WholebodyControlNode
//
// MoveIt의 FollowJointTrajectory 액션 서버를 구현.
// hardware_node가 이미 PowerOn/ServoOn/EnableControlManager를 처리했으므로
// 이 노드는 별도 gRPC 연결 후 CommandStream만 생성해서 명령을 전송한다.
//
// 시작 순서:
//   1. /joint_states 첫 수신 → hardware_node 초기화 완료 신호
//   2. robot Connect() + CreateCommandStream()
//   3. FollowJointTrajectory 액션 골 수락 시작
// ─────────────────────────────────────────────────────────────────────────────
class RBY1WholebodyControlNode : public rclcpp::Node {
  using FJT         = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;
  using RobotT      = rb::Robot<rb::y1_model::A>;
  using StreamT     = rb::RobotCommandStreamHandler<rb::y1_model::A>;

 public:
  explicit RBY1WholebodyControlNode(const std::string& address)
      : Node("control_moveit_node"), address_(address) {
    // /joint_states 구독 → hardware_node 준비 완료 감지 + 현재 관절 위치 유지
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_mutex_);
          for (size_t i = 0; i < msg->name.size(); ++i) {
            current_positions_[msg->name[i]] = msg->position[i];
          }
          if (!hardware_ready_.exchange(true)) {
            RCLCPP_INFO(get_logger(),
                        "/joint_states 수신 확인 → 로봇 연결 시작");
            // 연결은 별도 스레드에서 (콜백 블로킹 방지)
            std::thread([this]() { connect_robot(); }).detach();
          }
        });

    // FollowJointTrajectory 액션 서버 생성
    // 네임: rby1_wholebody_controller/follow_joint_trajectory
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FJT>(
        this,
        "rby1_wholebody_controller/follow_joint_trajectory",
        std::bind(&RBY1WholebodyControlNode::handle_goal,    this, _1, _2),
        std::bind(&RBY1WholebodyControlNode::handle_cancel,  this, _1),
        std::bind(&RBY1WholebodyControlNode::handle_accepted,this, _1));

    RCLCPP_INFO(get_logger(),
                "Wholebody control node 시작. /joint_states 대기 중...");
  }

  ~RBY1WholebodyControlNode() {
    cancel_requested_ = true;
  }

 private:
  // ── 로봇 연결 ─────────────────────────────────────────────────────────────
  void connect_robot() {
    robot_ = RobotT::Create(address_);

    RCLCPP_INFO(get_logger(), "로봇 연결 중: %s", address_.c_str());
    if (!robot_->Connect()) {
      RCLCPP_FATAL(get_logger(), "로봇 연결 실패: %s", address_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "로봇 연결 완료");

    stream_ready_ = true;
    RCLCPP_INFO(get_logger(),
                "로봇 연결 완료. FollowJointTrajectory 골 수락 시작.");
  }

  // ── 액션 핸들러 ───────────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const FJT::Goal> goal) {
    if (!stream_ready_) {
      RCLCPP_WARN(get_logger(), "로봇 미준비 — 골 거절");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->trajectory.joint_names.empty() ||
        goal->trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "빈 궤적 — 골 거절");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(),
                "궤적 골 수락: %zu 포인트, %zu 조인트",
                goal->trajectory.points.size(),
                goal->trajectory.joint_names.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFJT>) {
    RCLCPP_INFO(get_logger(), "취소 요청 수신");
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle) {
    // 별도 스레드에서 실행 (액션 콜백 블로킹 방지)
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // ── 궤적 실행 ─────────────────────────────────────────────────────────────
  void execute(std::shared_ptr<GoalHandleFJT> goal_handle) {
    cancel_requested_ = false;

    // 매 실행 전 stream 재생성: 이전 trajectory 후 SDK가 stream을 만료시키므로
    try {
      stream_ = robot_->CreateCommandStream(2);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "CommandStream 재생성 실패: %s", e.what());
      auto result = std::make_shared<FJT::Result>();
      result->error_code = FJT::Result::INVALID_GOAL;
      goal_handle->abort(result);
      return;
    }

    const auto& traj        = goal_handle->get_goal()->trajectory;
    const auto& joint_names = traj.joint_names;
    const size_t n_points   = traj.points.size();

    auto feedback = std::make_shared<FJT::Feedback>();
    auto result   = std::make_shared<FJT::Result>();

    // 궤적 시작 기준 시각
    const rclcpp::Time traj_start = now();

    for (size_t pt_idx = 0; pt_idx < n_points; ++pt_idx) {
      if (cancel_requested_) {
        result->error_code = FJT::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "궤적 취소됨 (포인트 %zu/%zu)", pt_idx, n_points);
        stream_ = nullptr;
        return;
      }

      const auto& point = traj.points[pt_idx];

      // ── Eigen 벡터 초기화 (현재 관절 위치로 seeding) ──────────────────────
      Eigen::Vector<double, 6> torso;
      Eigen::Vector<double, 7> right_arm;
      Eigen::Vector<double, 7> left_arm;
      Eigen::Vector<double, 2> head;

      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        torso[0] = current_positions_.count("torso_0")     ? current_positions_["torso_0"]     : 0.0;
        torso[1] = current_positions_.count("torso_1")     ? current_positions_["torso_1"]     : 0.0;
        torso[2] = current_positions_.count("torso_2")     ? current_positions_["torso_2"]     : 0.0;
        torso[3] = current_positions_.count("torso_3")     ? current_positions_["torso_3"]     : 0.0;
        torso[4] = current_positions_.count("torso_4")     ? current_positions_["torso_4"]     : 0.0;
        torso[5] = current_positions_.count("torso_5")     ? current_positions_["torso_5"]     : 0.0;

        right_arm[0] = current_positions_.count("right_arm_0") ? current_positions_["right_arm_0"] : 0.0;
        right_arm[1] = current_positions_.count("right_arm_1") ? current_positions_["right_arm_1"] : 0.0;
        right_arm[2] = current_positions_.count("right_arm_2") ? current_positions_["right_arm_2"] : 0.0;
        right_arm[3] = current_positions_.count("right_arm_3") ? current_positions_["right_arm_3"] : 0.0;
        right_arm[4] = current_positions_.count("right_arm_4") ? current_positions_["right_arm_4"] : 0.0;
        right_arm[5] = current_positions_.count("right_arm_5") ? current_positions_["right_arm_5"] : 0.0;
        right_arm[6] = current_positions_.count("right_arm_6") ? current_positions_["right_arm_6"] : 0.0;

        left_arm[0] = current_positions_.count("left_arm_0") ? current_positions_["left_arm_0"] : 0.0;
        left_arm[1] = current_positions_.count("left_arm_1") ? current_positions_["left_arm_1"] : 0.0;
        left_arm[2] = current_positions_.count("left_arm_2") ? current_positions_["left_arm_2"] : 0.0;
        left_arm[3] = current_positions_.count("left_arm_3") ? current_positions_["left_arm_3"] : 0.0;
        left_arm[4] = current_positions_.count("left_arm_4") ? current_positions_["left_arm_4"] : 0.0;
        left_arm[5] = current_positions_.count("left_arm_5") ? current_positions_["left_arm_5"] : 0.0;
        left_arm[6] = current_positions_.count("left_arm_6") ? current_positions_["left_arm_6"] : 0.0;

        head[0] = current_positions_.count("head_0") ? current_positions_["head_0"] : 0.0;
        head[1] = current_positions_.count("head_1") ? current_positions_["head_1"] : 0.0;
      }

      // ── 궤적 포인트의 관절 위치로 덮어쓰기 ───────────────────────────────
      for (size_t j = 0; j < joint_names.size(); ++j) {
        const auto& name = joint_names[j];
        const double pos = point.positions[j];

        if      (name == "torso_0") torso[0] = pos;
        else if (name == "torso_1") torso[1] = pos;
        else if (name == "torso_2") torso[2] = pos;
        else if (name == "torso_3") torso[3] = pos;
        else if (name == "torso_4") torso[4] = pos;
        else if (name == "torso_5") torso[5] = pos;

        else if (name == "right_arm_0") right_arm[0] = pos;
        else if (name == "right_arm_1") right_arm[1] = pos;
        else if (name == "right_arm_2") right_arm[2] = pos;
        else if (name == "right_arm_3") right_arm[3] = pos;
        else if (name == "right_arm_4") right_arm[4] = pos;
        else if (name == "right_arm_5") right_arm[5] = pos;
        else if (name == "right_arm_6") right_arm[6] = pos;

        else if (name == "left_arm_0") left_arm[0] = pos;
        else if (name == "left_arm_1") left_arm[1] = pos;
        else if (name == "left_arm_2") left_arm[2] = pos;
        else if (name == "left_arm_3") left_arm[3] = pos;
        else if (name == "left_arm_4") left_arm[4] = pos;
        else if (name == "left_arm_5") left_arm[5] = pos;
        else if (name == "left_arm_6") left_arm[6] = pos;

        else if (name == "head_0") head[0] = pos;
        else if (name == "head_1") head[1] = pos;
      }

      // ── min_time 계산: 현재-다음 포인트 간격 ─────────────────────────────
      const double cur_time  = rclcpp::Duration(point.time_from_start).seconds();
      double min_time = 0.03;  // 최소 30ms
      if (pt_idx + 1 < n_points) {
        const double next_time =
            rclcpp::Duration(traj.points[pt_idx + 1].time_from_start).seconds();
        min_time = std::max(0.03, next_time - cur_time);
      }
      // hold_time = min_time + 여유 (다음 명령이 도착할 때까지 위치 유지)
      const double hold_time = min_time + 0.3;

      // ── SDK 명령 전송 ─────────────────────────────────────────────────────
      try {
        stream_->SendCommand(
            rb::RobotCommandBuilder().SetCommand(
                rb::ComponentBasedCommandBuilder()
                    .SetBodyCommand(
                        rb::BodyComponentBasedCommandBuilder()
                            .SetRightArmCommand(
                                rb::JointPositionCommandBuilder()
                                    .SetCommandHeader(
                                        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                                    .SetMinimumTime(min_time)
                                    .SetPosition(right_arm))
                            .SetLeftArmCommand(
                                rb::JointPositionCommandBuilder()
                                    .SetCommandHeader(
                                        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                                    .SetMinimumTime(min_time)
                                    .SetPosition(left_arm))
                            .SetTorsoCommand(
                                rb::JointPositionCommandBuilder()
                                    .SetCommandHeader(
                                        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                                    .SetMinimumTime(min_time)
                                    .SetPosition(torso)))
                    .SetHeadCommand(
                        rb::JointPositionCommandBuilder()
                            .SetCommandHeader(
                                rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                            .SetMinimumTime(min_time)
                            .SetPosition(head))));
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "SendCommand 실패: %s", e.what());
        result->error_code = FJT::Result::PATH_TOLERANCE_VIOLATED;
        goal_handle->abort(result);
        stream_ = nullptr;
        return;
      }

      // ── 피드백 발행 ───────────────────────────────────────────────────────
      feedback->joint_names = joint_names;
      feedback->desired     = point;
      goal_handle->publish_feedback(feedback);

      // ── 다음 포인트 시각까지 대기 ─────────────────────────────────────────
      if (pt_idx + 1 < n_points) {
        const double next_time =
            rclcpp::Duration(traj.points[pt_idx + 1].time_from_start).seconds();
        const auto target_time =
            traj_start + rclcpp::Duration::from_seconds(next_time);
        std::this_thread::sleep_until(
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>((target_time - now()).seconds()));
      }
    }

    result->error_code = FJT::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "궤적 실행 완료 (%zu 포인트)", n_points);

    // 스트림 해제 → mobility(priority=1)에 제어권 반환
    stream_ = nullptr;
  }

  // ── 멤버 변수 ─────────────────────────────────────────────────────────────
  std::string address_;

  std::shared_ptr<RobotT>  robot_;
  std::unique_ptr<StreamT> stream_;

  std::atomic<bool> hardware_ready_{false};
  std::atomic<bool> stream_ready_{false};
  std::atomic<bool> cancel_requested_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Server<FJT>::SharedPtr action_server_;

  std::mutex joint_mutex_;
  std::map<std::string, double> current_positions_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("control_moveit_node_param_reader");
  param_node->declare_parameter<std::string>("robot_address", "192.168.12.1:50051");

  const auto address = param_node->get_parameter("robot_address").as_string();

  try {
    auto node = std::make_shared<RBY1WholebodyControlNode>(address);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(param_node->get_logger(),
                 "Wholebody control node 오류: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
