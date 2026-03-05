/**
 * go2_loco_bridge_node.cpp
 *
 * Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards velocity commands
 * to the Unitree Go2's high-level API topic (/api/sport/request).
 *
 * API ID 1008, JSON parameter: {"x": vx, "y": vy, "z": vyaw}
 *
 * Usage:
 *   ros2 run unitree_utils go2_loco_bridge_node
 */

#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

constexpr int32_t GO2_API_ID_MOVE = 1008;

class Go2LocoBridgeNode : public rclcpp::Node {
 public:
  Go2LocoBridgeNode() : Node("go2_loco_bridge_node") {
    publisher_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          OnCmdVel(msg);
        });

    RCLCPP_INFO(this->get_logger(),
                "go2_loco_bridge_node started. /cmd_vel -> /api/sport/request");
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const float vx = static_cast<float>(msg->linear.x);
    const float vy = static_cast<float>(msg->linear.y);
    const float vyaw = static_cast<float>(msg->angular.z);

    // api_id=1008, parameter={"x":vx,"y":vy,"z":vyaw}
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;

    unitree_api::msg::Request req;
    req.parameter = js.dump();
    req.header.identity.api_id = GO2_API_ID_MOVE;
    publisher_->publish(req);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2LocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
