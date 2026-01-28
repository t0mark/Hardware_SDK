#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "inspire_hand_eth/modbus_client.hpp"
#include "inspire_hand_eth/register_map.hpp"

// Message interfaces
#include "service_interfaces/msg/get_angle_act1.hpp"
#include "service_interfaces/msg/get_force_act1.hpp"
#include "service_interfaces/msg/get_touch_act1.hpp"
#include "service_interfaces/msg/get_temp1.hpp"
#include "service_interfaces/msg/set_angle1.hpp"
#include "service_interfaces/msg/set_force1.hpp"
#include "service_interfaces/msg/set_speed1.hpp"

using namespace inspire_hand_eth;
using namespace std::chrono_literals;

class TopicNode : public rclcpp::Node
{
public:
    TopicNode()
    : Node("inspire_hand_topic_node")
    {
        // Load config.yaml
        std::string pkg_share = ament_index_cpp::get_package_share_directory("inspire_hand_eth");
        std::string config_path = pkg_share + "/config/config.yaml";

        YAML::Node config = YAML::LoadFile(config_path);
        std::string ip = config["inspire_hand"]["modbus"]["ip"].as<std::string>();
        int port = config["inspire_hand"]["modbus"]["port"].as<int>();
        double publish_rate = config["inspire_hand"]["topic"]["publish_rate_hz"].as<double>(10.0);

        RCLCPP_INFO(this->get_logger(), "Config loaded: %s:%d, rate: %.1f Hz",
                    ip.c_str(), port, publish_rate);

        // Create Modbus client and connect
        modbus_client_ = std::make_shared<ModbusClient>(ip, port, this->get_logger());

        if (!modbus_client_->connect()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to Modbus TCP server");
            rclcpp::shutdown();
            return;
        }

        // Initialize finger names
        for (int i = 0; i < NUM_FINGERS; ++i) {
            finger_names_.push_back(FINGER_NAMES[i]);
        }

        // Publishers
        pub_angle_ = this->create_publisher<service_interfaces::msg::GetAngleAct1>("angle_data", 10);
        pub_force_ = this->create_publisher<service_interfaces::msg::GetForceAct1>("force_data", 10);
        pub_touch_ = this->create_publisher<service_interfaces::msg::GetTouchAct1>("touch_data", 10);
        pub_temp_ = this->create_publisher<service_interfaces::msg::GetTemp1>("temp_data", 10);

        // Subscribers
        sub_angle_ = this->create_subscription<service_interfaces::msg::SetAngle1>(
            "set_angle_data", 10,
            std::bind(&TopicNode::cb_set_angle, this, std::placeholders::_1));

        sub_force_ = this->create_subscription<service_interfaces::msg::SetForce1>(
            "set_force_data", 10,
            std::bind(&TopicNode::cb_set_force, this, std::placeholders::_1));

        sub_speed_ = this->create_subscription<service_interfaces::msg::SetSpeed1>(
            "set_speed_data", 10,
            std::bind(&TopicNode::cb_set_speed, this, std::placeholders::_1));

        // Timer for publishing
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TopicNode::publish_all_data, this));

        RCLCPP_INFO(this->get_logger(), "Topic node started");
        RCLCPP_INFO(this->get_logger(), "  Publishers: angle_data, force_data, touch_data, temp_data");
        RCLCPP_INFO(this->get_logger(), "  Subscribers: set_angle_data, set_force_data, set_speed_data");
    }

private:
    std::shared_ptr<ModbusClient> modbus_client_;
    std::vector<std::string> finger_names_;

    // Publishers
    rclcpp::Publisher<service_interfaces::msg::GetAngleAct1>::SharedPtr pub_angle_;
    rclcpp::Publisher<service_interfaces::msg::GetForceAct1>::SharedPtr pub_force_;
    rclcpp::Publisher<service_interfaces::msg::GetTouchAct1>::SharedPtr pub_touch_;
    rclcpp::Publisher<service_interfaces::msg::GetTemp1>::SharedPtr pub_temp_;

    // Subscribers
    rclcpp::Subscription<service_interfaces::msg::SetAngle1>::SharedPtr sub_angle_;
    rclcpp::Subscription<service_interfaces::msg::SetForce1>::SharedPtr sub_force_;
    rclcpp::Subscription<service_interfaces::msg::SetSpeed1>::SharedPtr sub_speed_;

    rclcpp::TimerBase::SharedPtr timer_;

    void publish_all_data()
    {
        // Only read and publish if there are subscribers
        if (pub_angle_->get_subscription_count() > 0) {
            publish_angle_data();
        }
        if (pub_force_->get_subscription_count() > 0) {
            publish_force_data();
        }
        if (pub_touch_->get_subscription_count() > 0) {
            publish_touch_data();
        }
        if (pub_temp_->get_subscription_count() > 0) {
            publish_temp_data();
        }
    }

    void publish_angle_data()
    {
        auto msg = service_interfaces::msg::GetAngleAct1();
        std::vector<int16_t> values;

        if (modbus_client_->readNonContiguousRegisters(REG_ANGLE_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                msg.finger_ids.push_back(i + 1);
                msg.angles.push_back(values[i]);
                msg.finger_names.push_back(finger_names_[i]);
            }
            pub_angle_->publish(msg);
        }
    }

    void publish_force_data()
    {
        auto msg = service_interfaces::msg::GetForceAct1();
        std::vector<int16_t> values;

        if (modbus_client_->readNonContiguousRegisters(REG_FORCE_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                msg.finger_ids.push_back(i + 1);
                msg.force_values.push_back(values[i]);
                msg.finger_names.push_back(finger_names_[i]);
            }
            pub_force_->publish(msg);
        }
    }

    void publish_touch_data()
    {
        auto msg = service_interfaces::msg::GetTouchAct1();

        // Touch sensor ranges for each finger/palm
        std::vector<TouchRange> touch_ranges = {
            TOUCH_PINKY, TOUCH_RING, TOUCH_MIDDLE,
            TOUCH_INDEX, TOUCH_THUMB, TOUCH_PALM
        };
        std::vector<std::string> touch_names = {
            "Pinky", "Ring", "Middle", "Index", "Thumb", "Palm"
        };

        for (size_t i = 0; i < touch_ranges.size(); ++i) {
            int count = (touch_ranges[i].end - touch_ranges[i].start) / 2 + 1;
            std::vector<int16_t> values;

            if (modbus_client_->readRegisters(touch_ranges[i].start, count, values)) {
                msg.finger_ids.push_back(static_cast<int>(i + 1));
                msg.finger_names.push_back(touch_names[i]);
                for (const auto & v : values) {
                    msg.touch_values.push_back(v);
                }
            }
        }

        if (!msg.finger_ids.empty()) {
            pub_touch_->publish(msg);
        }
    }

    void publish_temp_data()
    {
        auto msg = service_interfaces::msg::GetTemp1();
        std::vector<int16_t> values;

        if (modbus_client_->readRegisters(REG_TEMP_START, NUM_FINGERS, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                msg.finger_ids.push_back(i + 1);
                // Use lower 8 bits only (same as original Python code)
                msg.temp_values.push_back(values[i] & 0xFF);
                msg.finger_names.push_back(finger_names_[i]);
            }
            pub_temp_->publish(msg);
        }
    }

    // ===== Subscriber Callbacks =====

    void cb_set_angle(const service_interfaces::msg::SetAngle1::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received set_angle_data");

        for (size_t i = 0; i < msg->finger_ids.size() && i < msg->angles.size(); ++i) {
            int finger_id = msg->finger_ids[i];
            int angle = msg->angles[i];

            if (finger_id >= 1 && finger_id <= NUM_FINGERS &&
                angle >= 0 && angle <= 1000)
            {
                modbus_client_->writeRegister(
                    REG_ANGLE_SET[finger_id - 1],
                    static_cast<uint16_t>(angle));
            }
        }
    }

    void cb_set_force(const service_interfaces::msg::SetForce1::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received set_force_data");

        for (size_t i = 0; i < msg->finger_ids.size() && i < msg->forces.size(); ++i) {
            int finger_id = msg->finger_ids[i];
            int force = msg->forces[i];

            if (finger_id >= 1 && finger_id <= NUM_FINGERS &&
                force >= 0 && force <= 3000)
            {
                modbus_client_->writeRegister(
                    REG_FORCE_SET[finger_id - 1],
                    static_cast<uint16_t>(force));
            }
        }
    }

    void cb_set_speed(const service_interfaces::msg::SetSpeed1::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received set_speed_data");

        for (size_t i = 0; i < msg->finger_ids.size() && i < msg->speeds.size(); ++i) {
            int finger_id = msg->finger_ids[i];
            int speed = msg->speeds[i];

            if (finger_id >= 1 && finger_id <= NUM_FINGERS &&
                speed >= 0 && speed <= 1000)
            {
                modbus_client_->writeRegister(
                    REG_SPEED_SET[finger_id - 1],
                    static_cast<uint16_t>(speed));
            }
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TopicNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
