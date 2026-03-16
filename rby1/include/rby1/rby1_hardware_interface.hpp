#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rby1
{

class RBY1HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RBY1HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string robot_ip_;

  // gRPC channel and stubs (opaque to keep heavy headers out of here)
  struct GrpcStubs;
  std::unique_ptr<GrpcStubs> grpc_;

  // Joint state (indexed by info_.joints order)
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_commands_;

  // Background state streaming thread
  std::thread       state_thread_;
  std::atomic<bool> state_running_{false};
  std::mutex        state_mutex_;
  bool              state_received_{false};

  // Mobile base velocity (updated by /cmd_vel subscriber)
  std::mutex vel_mutex_;
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_omega_{0.0};

  // ROS node for subscriptions inside the hardware interface
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  bool cmd_stream_initialized_{false};

  // Helpers
  bool checkIsPowerOn();
  bool checkIsServoOn();
  bool sendPowerOn(const std::string & name);
  bool sendServoOn(const std::string & name);
  bool sendControlManagerCommand(int command);
  void stateStreamLoop();
  void sendWriteCommand();
};

}  // namespace rby1
