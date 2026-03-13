#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "inspire_hand_eth/modbus_client.hpp"
#include "inspire_hand_eth/register_map.hpp"

using namespace inspire_hand_eth;

class TestNode : public rclcpp::Node
{
public:
    TestNode()
    : Node("inspire_hand_test_node")
    {
        // Load config.yaml
        std::string pkg_share = ament_index_cpp::get_package_share_directory("inspire_hand_eth");
        std::string config_path = pkg_share + "/config/config.yaml";

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "  Inspire Hand Modbus TCP Connection Test  ");
        RCLCPP_INFO(this->get_logger(), "===========================================");

        try {
            YAML::Node config = YAML::LoadFile(config_path);
            std::string ip = config["inspire_hand"]["modbus"]["ip"].as<std::string>();
            int port = config["inspire_hand"]["modbus"]["port"].as<int>();

            RCLCPP_INFO(this->get_logger(), "Config loaded from: %s", config_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Target: %s:%d", ip.c_str(), port);
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------");

            // Create Modbus client and test connection
            modbus_client_ = std::make_shared<ModbusClient>(ip, port, this->get_logger());

            if (modbus_client_->connect()) {
                RCLCPP_INFO(this->get_logger(), "[SUCCESS] Connection established!");

                // Simple read test (current angles)
                RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "Testing register read (current angles)...");

                std::vector<int16_t> angles;
                if (modbus_client_->readNonContiguousRegisters(REG_ANGLE_ACT, angles)) {
                    RCLCPP_INFO(this->get_logger(), "[SUCCESS] Read test passed!");
                    for (int i = 0; i < NUM_FINGERS; ++i) {
                        RCLCPP_INFO(this->get_logger(), "  %s angle: %d",
                                    FINGER_NAMES[i], angles[i]);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[FAILED] Read test failed!");
                }

                modbus_client_->disconnect();
            } else {
                RCLCPP_ERROR(this->get_logger(), "[FAILED] Connection failed!");
            }
        } catch (const YAML::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "              Test Complete                ");
        RCLCPP_INFO(this->get_logger(), "===========================================");

        rclcpp::shutdown();
    }

private:
    std::shared_ptr<ModbusClient> modbus_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::shutdown();
    return 0;
}
