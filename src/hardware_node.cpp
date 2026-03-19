#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>

// ─────────────────────────────────────────────────────────────────────────────
// RBY1HardwareNode
//
// 하드웨어 연결 → 전원/서보 ON → control manager 활성화 → joint states 발행
// ─────────────────────────────────────────────────────────────────────────────
template <typename ModelT>
class RBY1HardwareNode : public rclcpp::Node {
  using RobotT = rb::Robot<ModelT>;
  using StateT = rb::RobotState<ModelT>;

 public:
  explicit RBY1HardwareNode(const std::string& address, double rate)
      : Node("rby1_hardware"), address_(address), rate_(rate) {
    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    initialize_robot();
  }

  ~RBY1HardwareNode() {
    if (robot_) {
      robot_->StopStateUpdate();
    }
  }

 private:
  // ── 초기화 ──────────────────────────────────────────────────────────────────
  void initialize_robot() {
    robot_ = RobotT::Create(address_);

    RCLCPP_INFO(get_logger(), "Connecting to robot at %s ...", address_.c_str());
    if (!robot_->Connect()) {
      RCLCPP_FATAL(get_logger(), "Failed to connect to robot");
      throw std::runtime_error("Failed to connect to robot");
    }
    RCLCPP_INFO(get_logger(), "Connected");

    // SDK 예제(demo_motion.cpp)와 동일한 순서:
    // StartStateUpdate → sleep → power → servo → fault reset → enable

    robot_->StartStateUpdate(
        [this](const StateT& state) { state_callback(state); }, rate_);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 전원
    if (!robot_->IsPowerOn(".*")) {
      RCLCPP_INFO(get_logger(), "Powering on ...");
      if (!robot_->PowerOn(".*")) {
        RCLCPP_FATAL(get_logger(), "Failed to power on");
        throw std::runtime_error("Failed to power on");
      }
    }
    RCLCPP_INFO(get_logger(), "Power on");

    // 서보
    if (!robot_->IsServoOn(".*")) {
      RCLCPP_INFO(get_logger(), "Servo on ...");
      if (!robot_->ServoOn(".*")) {
        RCLCPP_FATAL(get_logger(), "Failed to servo on");
        throw std::runtime_error("Failed to servo on");
      }
    }
    RCLCPP_INFO(get_logger(), "Servo on");

    // Control manager: fault이면 reset 후 enable
    const auto& cm = robot_->GetControlManagerState();
    if (cm.state == rb::ControlManagerState::State::kMajorFault ||
        cm.state == rb::ControlManagerState::State::kMinorFault) {
      RCLCPP_WARN(get_logger(), "Control manager fault (%s), resetting ...",
                  rb::to_string(cm.state).c_str());
      if (!robot_->ResetFaultControlManager()) {
        RCLCPP_FATAL(get_logger(), "Failed to reset control manager fault");
        throw std::runtime_error("Failed to reset control manager fault");
      }
    }

    if (!robot_->EnableControlManager()) {
      RCLCPP_FATAL(get_logger(), "Failed to enable control manager");
      throw std::runtime_error("Failed to enable control manager");
    }
    RCLCPP_INFO(get_logger(), "Control manager enabled. Robot ready.");
  }

  // ── State 콜백 ───────────────────────────────────────────────────────────────
  void state_callback(const StateT& state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();

    msg.name.reserve(ModelT::kRobotDOF);
    msg.position.reserve(ModelT::kRobotDOF);
    msg.velocity.reserve(ModelT::kRobotDOF);

    for (const auto& name : ModelT::kRobotJointNames) {
      msg.name.emplace_back(name);
    }
    for (size_t i = 0; i < ModelT::kRobotDOF; ++i) {
      msg.position.push_back(state.position[i]);
      msg.velocity.push_back(state.velocity[i]);
    }

    joint_state_pub_->publish(msg);
  }

  // ── 멤버 ─────────────────────────────────────────────────────────────────────
  std::string address_;
  double rate_;
  std::shared_ptr<RobotT> robot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 파라미터는 rclcpp::Node 생성 전에 node options로 오버라이드 가능하나,
  // 여기서는 간단히 임시 노드로 읽는다.
  auto param_node = rclcpp::Node::make_shared("rby1_hardware_param_reader");
  param_node->declare_parameter<std::string>("robot_address", "192.168.30.1:50051");
  param_node->declare_parameter<std::string>("model", "a");
  param_node->declare_parameter<double>("rate", 50.0);

  const auto address = param_node->get_parameter("robot_address").as_string();
  const auto model   = param_node->get_parameter("model").as_string();
  const auto rate    = param_node->get_parameter("rate").as_double();

  try {
    if (model == "a") {
      auto node = std::make_shared<RBY1HardwareNode<rb::y1_model::A>>(address, rate);
      rclcpp::spin(node);
    } else if (model == "m") {
      auto node = std::make_shared<RBY1HardwareNode<rb::y1_model::M>>(address, rate);
      rclcpp::spin(node);
    } else if (model == "ub") {
      auto node = std::make_shared<RBY1HardwareNode<rb::y1_model::UB>>(address, rate);
      rclcpp::spin(node);
    } else {
      RCLCPP_FATAL(param_node->get_logger(), "Unknown model: '%s'. Use 'a', 'm', or 'ub'.", model.c_str());
      return 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(param_node->get_logger(), "Hardware node error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
