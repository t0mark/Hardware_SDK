/**
 * g1_loco_bridge_node.cpp
 *
 * Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards velocity commands
 * to the Unitree G1's high-level API topic (/api/sport/request).
 *
 * API ID 7105, JSON parameter: {"velocity": [vx, vy, omega], "duration": 1.0}
 *
 * Usage:
 *   ros2 run unitree_utils g1_loco_bridge_node
 */

#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

constexpr int32_t G1_API_ID_SET_VELOCITY = 7105;

class G1LocoBridgeNode : public rclcpp::Node {
 public:
  G1LocoBridgeNode() : Node("g1_loco_bridge_node") {
    publisher_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          OnCmdVel(msg);
        });

    RCLCPP_INFO(this->get_logger(),
                "g1_loco_bridge_node started. /cmd_vel -> /api/sport/request");
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const float vx = static_cast<float>(msg->linear.x);
    const float vy = static_cast<float>(msg->linear.y);
    const float omega = static_cast<float>(msg->angular.z);

    // api_id=7105, parameter={"velocity":[vx,vy,omega],"duration":1.0}
    nlohmann::json js;
    js["velocity"] = {vx, vy, omega};
    js["duration"] = 1.0F;

    unitree_api::msg::Request req;
    req.parameter = js.dump();
    req.header.identity.api_id = G1_API_ID_SET_VELOCITY;
    publisher_->publish(req);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1LocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
