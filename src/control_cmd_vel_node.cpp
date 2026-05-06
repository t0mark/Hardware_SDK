#include <chrono>
#include <limits>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
  RBY1MobilityNode()
      : Node("control_cmd_vel_node") {
    // 표준 ROS2 패턴: 생성자에서 파라미터 선언 및 읽기
    declare_parameter("robot_ip",            "rby1.local:50051");
    declare_parameter("control_hz",          10.0);
    declare_parameter("cmd_vel_timeout",     0.5);
    declare_parameter("avoidance",           false);
    declare_parameter("avoidance_distance",  1.0);
    declare_parameter("avoidance_turn_speed",0.5);
    declare_parameter("robot_half_width",    0.35);

    address_             = get_parameter("robot_ip").as_string();
    const double control_hz = get_parameter("control_hz").as_double();
    min_time_            = 1.0 / control_hz;
    cmd_vel_timeout_     = get_parameter("cmd_vel_timeout").as_double();
    avoidance_           = get_parameter("avoidance").as_bool();
    avoidance_dist_      = get_parameter("avoidance_distance").as_double();
    avoidance_turn_speed_= get_parameter("avoidance_turn_speed").as_double();
    robot_half_width_    = get_parameter("robot_half_width").as_double();

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

    if (avoidance_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10,
          [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            latest_scan_ = *msg;
            has_scan_ = true;
          });
      RCLCPP_INFO(get_logger(),
                  "Avoidance enabled (lookahead=%.2fm, half_width=%.2fm, turn_speed=%.2f rad/s)",
                  avoidance_dist_, robot_half_width_, avoidance_turn_speed_);
    }

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

  // ── LaserScan 기반 장애물 회피 (로봇 폭 기준 직사각형 충돌 판정) ──────────
  void apply_avoidance(geometry_msgs::msg::Twist& twist) {
    if (!avoidance_ || twist.linear.x <= 0.0) return;

    sensor_msgs::msg::LaserScan scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!has_scan_) return;
      scan = latest_scan_;
    }

    const float hw       = static_cast<float>(robot_half_width_);
    const float lookahead = static_cast<float>(avoidance_dist_);

    bool  front_blocked = false;
    float left_gap  = std::numeric_limits<float>::infinity();
    float right_gap = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      float r = scan.ranges[i];
      if (r <= 0.0f || r > scan.range_max) continue;

      const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      const float px = r * std::cos(angle);
      const float py = r * std::sin(angle);

      // 전방 충돌 경로: 로봇 폭 이내, 0 ~ lookahead
      if (px > 0.0f && px <= lookahead && std::abs(py) <= hw)
        front_blocked = true;

      // 전방 반구에서 좌우 여유 공간 측정
      if (px > 0.0f && px <= lookahead) {
        if (py > hw)
          left_gap  = std::min(left_gap,  py - hw);
        else if (py < -hw)
          right_gap = std::min(right_gap, -py - hw);
      }
    }

    if (!front_blocked) return;

    const bool left_ok  = left_gap  > static_cast<float>(avoidance_dist_);
    const bool right_ok = right_gap > static_cast<float>(avoidance_dist_);

    if (!left_ok && !right_ok) {
      twist.linear.x  = 0.0;
      twist.angular.z = 0.0;
    } else {
      const bool go_left = left_ok && (!right_ok || left_gap >= right_gap);
      twist.angular.z = go_left ? avoidance_turn_speed_ : -avoidance_turn_speed_;
    }
  }

  // ── /cmd_vel 콜백 ─────────────────────────────────────────────────────────
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    latest_twist_  = *msg;
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
        return;
      }

      auto elapsed = (now() - last_cmd_time_).seconds();
      if (elapsed > cmd_vel_timeout_) {
        twist = geometry_msgs::msg::Twist{};
      } else {
        twist = latest_twist_;
      }
    }

    apply_avoidance(twist);

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

  bool avoidance_;
  double avoidance_dist_;
  double avoidance_turn_speed_;
  double robot_half_width_;

  std::shared_ptr<RobotT> robot_;
  std::unique_ptr<StreamT> stream_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist latest_twist_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_{};
  bool has_scan_{false};
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 템플릿 분기에 필요한 model만 임시 노드로 읽고 즉시 소멸
  std::string model;
  {
    auto tmp = rclcpp::Node::make_shared("control_cmd_vel_node_model_reader");
    tmp->declare_parameter("model", "a");
    model = tmp->get_parameter("model").as_string();
  }

  try {
    if (model == "a") {
      rclcpp::spin(std::make_shared<RBY1MobilityNode<rb::y1_model::A>>());
    } else if (model == "m") {
      rclcpp::spin(std::make_shared<RBY1MobilityNode<rb::y1_model::M>>());
    } else if (model == "ub") {
      rclcpp::spin(std::make_shared<RBY1MobilityNode<rb::y1_model::UB>>());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("control_cmd_vel_node"),
                   "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
      return 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_cmd_vel_node"),
                 "Mobility node error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
