#include <chrono>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <rby1-sdk/robot_command_builder.h>

// ─────────────────────────────────────────────────────────────────────────────
// RBY1MobilityNode
//
// /cmd_vel (geometry_msgs/Twist) 구독 → SE2VelocityCommand 전송
//
// 전제: hardware_node가 이미 전원/서보 ON 및 control manager 활성화를 완료한 상태.
// 이 노드는 connect 후 속도 명령만 담당한다.
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
class RBY1MobilityNode : public rclcpp::Node {
  using RobotT  = rb::Robot<ModelT>;
  using StreamT = rb::RobotCommandStreamHandler<ModelT>;

 public:
  explicit RBY1MobilityNode(const std::string& address, double control_hz,
                             double cmd_vel_timeout)
      : Node("control_cmd_vel_node"),
        address_(address),
        min_time_(1.0 / control_hz),
        cmd_vel_timeout_(cmd_vel_timeout) {
    // UB 모델은 모바일 베이스 없음
    if constexpr (ModelT::kMobilityIdx.size() == 0) {
      RCLCPP_WARN(get_logger(),
                  "Model UB has no mobile base. Mobility commands will be ignored.");
    }

    connect_robot();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_vel_callback(msg);
        });

    auto period = std::chrono::duration<double>(1.0 / control_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(),
                "Mobility node ready. Subscribed to /cmd_vel (timeout=%.2fs, "
                "control_hz=%.1f)",
                cmd_vel_timeout_, 1.0 / min_time_);
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

    stream_ = robot_->CreateCommandStream(1);
    RCLCPP_INFO(get_logger(), "CommandStream ready (priority=1)");
  }

  // ── /cmd_vel 콜백 ─────────────────────────────────────────────────────────
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    latest_twist_ = *msg;
    last_cmd_time_ = now();
  }

  // ── 주기적 명령 전송 ──────────────────────────────────────────────────────
  void timer_callback() {
    if constexpr (ModelT::kMobilityIdx.size() == 0) {
      return;  // UB 모델: 베이스 없음
    }

    geometry_msgs::msg::Twist twist;

    {
      std::lock_guard<std::mutex> lock(twist_mutex_);

      if (last_cmd_time_.nanoseconds() == 0) {
        // 아직 cmd_vel을 한 번도 받지 않음 → 전송 스킵
        return;
      }

      auto elapsed = (now() - last_cmd_time_).seconds();
      if (elapsed > cmd_vel_timeout_) {
        // 타임아웃: 정지 명령 전송
        twist = geometry_msgs::msg::Twist{};
      } else {
        twist = latest_twist_;
      }
    }

    Eigen::Vector2d lin;
    lin << twist.linear.x, twist.linear.y;
    double ang = twist.angular.z;

    Eigen::Vector2d lin_acc;
    lin_acc << 10.0, 10.0;
    double ang_acc = 10.0;

    auto send = [&]() {
      stream_->SendCommand(
          rb::RobotCommandBuilder().SetCommand(
              rb::ComponentBasedCommandBuilder().SetMobilityCommand(
                  rb::MobilityCommandBuilder().SetCommand(
                      rb::SE2VelocityCommandBuilder()
                          .SetVelocity(lin, ang)
                          .SetMinimumTime(min_time_)
                          .SetAccelerationLimit(lin_acc, ang_acc)))));
    };

    try {
      send();
    } catch (const std::exception&) {
      // 스트림 만료 시 재생성 후 재시도
      try {
        stream_ = robot_->CreateCommandStream(1);
        send();
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "SendCommand failed: %s", e.what());
      }
    }
  }

  // ── 멤버 ─────────────────────────────────────────────────────────────────
  std::string address_;
  double min_time_;
  double cmd_vel_timeout_;

  std::shared_ptr<RobotT> robot_;
  std::unique_ptr<StreamT> stream_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist latest_twist_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto param_node = rclcpp::Node::make_shared("control_cmd_vel_node_param_reader");
  param_node->declare_parameter<std::string>("robot_ip", "rby1.local:50051");
  param_node->declare_parameter<std::string>("model", "a");
  param_node->declare_parameter<double>("control_hz", 10.0);
  param_node->declare_parameter<double>("cmd_vel_timeout", 0.5);

  const auto address     = param_node->get_parameter("robot_ip").as_string();
  const auto model       = param_node->get_parameter("model").as_string();
  const auto control_hz  = param_node->get_parameter("control_hz").as_double();
  const auto timeout     = param_node->get_parameter("cmd_vel_timeout").as_double();

  try {
    if (model == "a") {
      auto node = std::make_shared<RBY1MobilityNode<rb::y1_model::A>>(
          address, control_hz, timeout);
      rclcpp::spin(node);
    } else if (model == "m") {
      auto node = std::make_shared<RBY1MobilityNode<rb::y1_model::M>>(
          address, control_hz, timeout);
      rclcpp::spin(node);
    } else if (model == "ub") {
      auto node = std::make_shared<RBY1MobilityNode<rb::y1_model::UB>>(
          address, control_hz, timeout);
      rclcpp::spin(node);
    } else {
      RCLCPP_FATAL(param_node->get_logger(),
                   "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
      return 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(param_node->get_logger(), "Mobility node error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
