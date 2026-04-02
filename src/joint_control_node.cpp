#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <rby1-sdk/robot_command_builder.h>

// ─────────────────────────────────────────────────────────────────────────────
// RBY1JointControlNode
//
// /joint_commands (sensor_msgs/JointState) 구독 → SDK CommandStream (priority=3)
//
// 상체 관절 (torso/arms/head):
//   JointState.position[] → JointPositionCommandBuilder
//
// 바퀴 관절 (control_base=true 시):
//   JointState.velocity[] → JointVelocityCommandBuilder
//   바퀴 이름은 ModelT::kRobotJointNames[kMobilityIdx[w]] 로 자동 결정
//
// 전제: hardware_node가 이미 PowerOn/ServoOn/EnableControlManager 완료.
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
class RBY1JointControlNode : public rclcpp::Node {
  using RobotT  = rb::Robot<ModelT>;
  using StreamT = rb::RobotCommandStreamHandler<ModelT>;
  static constexpr size_t kNWheels = ModelT::kMobilityIdx.size();

 public:
  explicit RBY1JointControlNode(const std::string& address,
                                 double control_hz,
                                 double cmd_timeout,
                                 bool   control_base)
      : Node("joint_control_node"),
        address_(address),
        min_time_(1.0 / control_hz),
        cmd_timeout_(cmd_timeout),
        control_base_(control_base && (kNWheels > 0)) {
    if (control_base && kNWheels == 0) {
      RCLCPP_WARN(get_logger(),
                  "control_base=true but model has no wheels. Base control disabled.");
    }

    connect_robot();

    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_commands", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          latest_cmd_   = *msg;
          last_cmd_time_ = now();
        });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size()) {
              current_positions_[msg->name[i]] = msg->position[i];
            }
          }
        });

    auto period = std::chrono::duration<double>(min_time_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(),
                "Joint control node ready (priority=3, control_base=%s, "
                "timeout=%.2fs, hz=%.1f)",
                control_base_ ? "true" : "false", cmd_timeout_, 1.0 / min_time_);
  }

 private:
  // ── 로봇 연결 및 스트림 생성 ───────────────────────────────────────────────
  void connect_robot() {
    robot_ = RobotT::Create(address_);
    RCLCPP_INFO(get_logger(), "Connecting to robot at %s ...", address_.c_str());
    if (!robot_->Connect()) {
      RCLCPP_FATAL(get_logger(), "Failed to connect to robot at %s", address_.c_str());
      throw std::runtime_error("Failed to connect to robot");
    }
    RCLCPP_INFO(get_logger(), "Connected");
    stream_ = robot_->CreateCommandStream(3);
    RCLCPP_INFO(get_logger(), "CommandStream ready (priority=3)");
  }

  // ── 주기적 명령 전송 ──────────────────────────────────────────────────────
  void timer_callback() {
    sensor_msgs::msg::JointState cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (last_cmd_time_.nanoseconds() == 0) return;  // 아직 명령 미수신
      if ((now() - last_cmd_time_).seconds() > cmd_timeout_) return;  // 타임아웃
      cmd = latest_cmd_;
    }

    // ── 상체 관절 벡터 초기화 (현재 위치로 seed) ─────────────────────────────
    Eigen::Vector<double, 6> torso;
    Eigen::Vector<double, 7> right_arm;
    Eigen::Vector<double, 7> left_arm;
    Eigen::Vector<double, 2> head;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto get = [&](const std::string& name) -> double {
        auto it = current_positions_.find(name);
        return it != current_positions_.end() ? it->second : 0.0;
      };
      for (int i = 0; i < 6; ++i) torso[i]     = get("torso_"     + std::to_string(i));
      for (int i = 0; i < 7; ++i) right_arm[i] = get("right_arm_" + std::to_string(i));
      for (int i = 0; i < 7; ++i) left_arm[i]  = get("left_arm_"  + std::to_string(i));
      head[0] = get("head_0");
      head[1] = get("head_1");
    }

    // ── 수신된 명령으로 덮어쓰기 ─────────────────────────────────────────────
    // kNWheels==0 일 때 zero-size array 회피를 위해 최소 크기 1 보장
    std::array<double, (kNWheels > 0 ? kNWheels : 1)> wheel_vel_arr{};
    bool has_wheel_cmd = false;

    for (size_t i = 0; i < cmd.name.size(); ++i) {
      const auto& name = cmd.name[i];

      // 바퀴 관절 체크 (control_base=true 이고 모델에 바퀴가 있을 때만)
      if constexpr (kNWheels > 0) {
        if (control_base_) {
          bool is_wheel = false;
          for (size_t w = 0; w < kNWheels; ++w) {
            if (name == std::string(ModelT::kRobotJointNames[ModelT::kMobilityIdx[w]])) {
              if (i < cmd.velocity.size()) {
                wheel_vel_arr[w] = cmd.velocity[i];
                has_wheel_cmd = true;
              }
              is_wheel = true;
              break;
            }
          }
          if (is_wheel) continue;
        }
      }

      // 상체 관절 매핑 (position 필드 사용)
      if (i >= cmd.position.size()) continue;
      const double pos = cmd.position[i];

      if      (name == "torso_0")     torso[0] = pos;
      else if (name == "torso_1")     torso[1] = pos;
      else if (name == "torso_2")     torso[2] = pos;
      else if (name == "torso_3")     torso[3] = pos;
      else if (name == "torso_4")     torso[4] = pos;
      else if (name == "torso_5")     torso[5] = pos;
      else if (name == "right_arm_0") right_arm[0] = pos;
      else if (name == "right_arm_1") right_arm[1] = pos;
      else if (name == "right_arm_2") right_arm[2] = pos;
      else if (name == "right_arm_3") right_arm[3] = pos;
      else if (name == "right_arm_4") right_arm[4] = pos;
      else if (name == "right_arm_5") right_arm[5] = pos;
      else if (name == "right_arm_6") right_arm[6] = pos;
      else if (name == "left_arm_0")  left_arm[0] = pos;
      else if (name == "left_arm_1")  left_arm[1] = pos;
      else if (name == "left_arm_2")  left_arm[2] = pos;
      else if (name == "left_arm_3")  left_arm[3] = pos;
      else if (name == "left_arm_4")  left_arm[4] = pos;
      else if (name == "left_arm_5")  left_arm[5] = pos;
      else if (name == "left_arm_6")  left_arm[6] = pos;
      else if (name == "head_0")      head[0] = pos;
      else if (name == "head_1")      head[1] = pos;
    }

    const double hold_time = min_time_ + 0.3;

    auto send = [&]() {
      auto cmd_builder =
          rb::ComponentBasedCommandBuilder()
              .SetBodyCommand(
                  rb::BodyComponentBasedCommandBuilder()
                      .SetTorsoCommand(
                          rb::JointPositionCommandBuilder()
                              .SetCommandHeader(
                                  rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                              .SetMinimumTime(min_time_)
                              .SetPosition(torso))
                      .SetRightArmCommand(
                          rb::JointPositionCommandBuilder()
                              .SetCommandHeader(
                                  rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                              .SetMinimumTime(min_time_)
                              .SetPosition(right_arm))
                      .SetLeftArmCommand(
                          rb::JointPositionCommandBuilder()
                              .SetCommandHeader(
                                  rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                              .SetMinimumTime(min_time_)
                              .SetPosition(left_arm)))
              .SetHeadCommand(
                  rb::JointPositionCommandBuilder()
                      .SetCommandHeader(
                          rb::CommandHeaderBuilder().SetControlHoldTime(hold_time))
                      .SetMinimumTime(min_time_)
                      .SetPosition(head));

      if constexpr (kNWheels > 0) {
        if (control_base_ && has_wheel_cmd) {
          Eigen::Map<const Eigen::VectorXd> wheel_vel(wheel_vel_arr.data(), kNWheels);
          cmd_builder.SetMobilityCommand(
              rb::MobilityCommandBuilder().SetCommand(
                  rb::JointVelocityCommandBuilder()
                      .SetVelocity(wheel_vel)
                      .SetMinimumTime(min_time_)));
        }
      }

      stream_->SendCommand(rb::RobotCommandBuilder().SetCommand(cmd_builder));
    };

    try {
      send();
    } catch (const std::exception&) {
      // 스트림 만료 시 재생성 후 재시도
      try {
        stream_ = robot_->CreateCommandStream(3);
        send();
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "SendCommand failed: %s", e.what());
      }
    }
  }

  // ── 멤버 변수 ─────────────────────────────────────────────────────────────
  std::string address_;
  double      min_time_;
  double      cmd_timeout_;
  bool        control_base_;

  std::shared_ptr<RobotT>  robot_;
  std::unique_ptr<StreamT> stream_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex cmd_mutex_;
  sensor_msgs::msg::JointState latest_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  std::mutex state_mutex_;
  std::map<std::string, double> current_positions_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("joint_control_param_reader");
  param_node->declare_parameter<std::string>("robot_address", "192.168.12.1:50051");
  param_node->declare_parameter<std::string>("model", "a");
  param_node->declare_parameter<double>("control_hz", 50.0);
  param_node->declare_parameter<double>("cmd_timeout", 0.5);
  param_node->declare_parameter<bool>("control_base", false);

  const auto address      = param_node->get_parameter("robot_address").as_string();
  const auto model        = param_node->get_parameter("model").as_string();
  const auto control_hz   = param_node->get_parameter("control_hz").as_double();
  const auto cmd_timeout  = param_node->get_parameter("cmd_timeout").as_double();
  const auto control_base = param_node->get_parameter("control_base").as_bool();

  try {
    if (model == "a") {
      auto node = std::make_shared<RBY1JointControlNode<rb::y1_model::A>>(
          address, control_hz, cmd_timeout, control_base);
      rclcpp::spin(node);
    } else if (model == "m") {
      auto node = std::make_shared<RBY1JointControlNode<rb::y1_model::M>>(
          address, control_hz, cmd_timeout, control_base);
      rclcpp::spin(node);
    } else if (model == "ub") {
      auto node = std::make_shared<RBY1JointControlNode<rb::y1_model::UB>>(
          address, control_hz, cmd_timeout, control_base);
      rclcpp::spin(node);
    } else {
      RCLCPP_FATAL(param_node->get_logger(),
                   "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
      return 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(param_node->get_logger(), "Joint control node error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
