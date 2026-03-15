#include "rby1/rby1_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <chrono>

// gRPC
#include <grpcpp/grpcpp.h>

// Generated proto headers (built at compile time)
#include "rb/api/robot_command_service.grpc.pb.h"
#include "rb/api/robot_state_service.grpc.pb.h"
#include "rb/api/power_service.grpc.pb.h"
#include "rb/api/control_manager_service.grpc.pb.h"

#include "rb/api/robot_command.pb.h"
#include "rb/api/component_based_command.pb.h"
#include "rb/api/body_command.pb.h"
#include "rb/api/body_component_based_command.pb.h"
#include "rb/api/arm_command.pb.h"
#include "rb/api/torso_command.pb.h"
#include "rb/api/head_command.pb.h"
#include "rb/api/mobility_command.pb.h"
#include "rb/api/basic_command.pb.h"
#include "rb/api/robot_state.pb.h"
#include "rb/api/power.pb.h"
#include "rb/api/control_manager.pb.h"

namespace rby1
{

// ---------------------------------------------------------------------------
// Internal struct holding all gRPC stubs and the command stream
// ---------------------------------------------------------------------------
struct RBY1HardwareInterface::GrpcStubs
{
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<rb::api::PowerService::Stub>          power;
  std::unique_ptr<rb::api::ControlManagerService::Stub> ctrl_mgr;
  std::unique_ptr<rb::api::RobotStateService::Stub>     state;
  std::unique_ptr<rb::api::RobotCommandService::Stub>   cmd;

  // Bidirectional streaming command channel
  std::unique_ptr<grpc::ClientContext> cmd_ctx;
  std::unique_ptr<grpc::ClientReaderWriter<
    rb::api::RobotCommandRequest,
    rb::api::RobotCommandResponse>> cmd_stream;
};

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------
static google::protobuf::Duration secToDuration(double sec)
{
  google::protobuf::Duration d;
  d.set_seconds(static_cast<int64_t>(sec));
  d.set_nanos(static_cast<int32_t>((sec - static_cast<int64_t>(sec)) * 1e9));
  return d;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn RBY1HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_ip_ = info.hardware_parameters.at("robot_ip");
  grpc_ = std::make_unique<GrpcStubs>();

  const size_t n = info.joints.size();
  joint_positions_.assign(n, 0.0);
  joint_velocities_.assign(n, 0.0);
  joint_commands_.assign(n, 0.0);

  node_ = std::make_shared<rclcpp::Node>("rby1_hw_node");

  RCLCPP_INFO(node_->get_logger(), "RBY1HardwareInterface initialized (%zu joints, ip=%s)",
              n, robot_ip_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  grpc_->channel  = grpc::CreateChannel(robot_ip_, grpc::InsecureChannelCredentials());
  grpc_->power    = rb::api::PowerService::NewStub(grpc_->channel);
  grpc_->ctrl_mgr = rb::api::ControlManagerService::NewStub(grpc_->channel);
  grpc_->state    = rb::api::RobotStateService::NewStub(grpc_->channel);
  grpc_->cmd      = rb::api::RobotCommandService::NewStub(grpc_->channel);

  RCLCPP_INFO(node_->get_logger(), "Connecting to robot at %s ...", robot_ip_.c_str());

  if (!sendPowerOn(".*")) {
    RCLCPP_ERROR(node_->get_logger(), "PowerOn failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!sendServoOn(".*")) {
    RCLCPP_ERROR(node_->get_logger(), "ServoOn failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  sendControlManagerCommand(3);  // RESET_FAULT (best-effort)
  if (!sendControlManagerCommand(1)) {  // ENABLE
    RCLCPP_ERROR(node_->get_logger(), "EnableControlManager failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node_->get_logger(), "Robot configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Subscribe to mobile base velocity
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      cmd_vx_    = msg->linear.x;
      cmd_vy_    = msg->linear.y;
      cmd_omega_ = msg->angular.z;
    });

  // Start state streaming thread
  state_running_ = true;
  state_thread_ = std::thread(&RBY1HardwareInterface::stateStreamLoop, this);

  // Wait up to 5 s for the first state
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!state_received_) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for first robot state");
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Seed commands from current positions
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_commands_ = joint_positions_;
  }

  // Open bidirectional command stream
  grpc_->cmd_ctx    = std::make_unique<grpc::ClientContext>();
  grpc_->cmd_stream = grpc_->cmd->RobotCommandStream(grpc_->cmd_ctx.get());
  cmd_stream_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Robot activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  state_running_ = false;
  if (state_thread_.joinable()) state_thread_.join();

  if (grpc_->cmd_stream) {
    grpc_->cmd_stream->WritesDone();
    grpc_->cmd_stream->Finish();
    grpc_->cmd_stream.reset();
  }
  grpc_->cmd_ctx.reset();
  cmd_stream_initialized_ = false;

  cmd_vel_sub_.reset();

  RCLCPP_INFO(node_->get_logger(), "Robot deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Export interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
RBY1HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
      }
    }
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
RBY1HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]);
      }
    }
  }
  return ifaces;
}

// ---------------------------------------------------------------------------
// read / write (called at 100 Hz by ros2_control_node)
// ---------------------------------------------------------------------------
hardware_interface::return_type RBY1HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);
  // joint_positions_ / joint_velocities_ are updated by stateStreamLoop()
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBY1HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!cmd_stream_initialized_ || !grpc_->cmd_stream) {
    return hardware_interface::return_type::OK;
  }
  sendWriteCommand();
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Background: gRPC state stream
// ---------------------------------------------------------------------------
void RBY1HardwareInterface::stateStreamLoop()
{
  rb::api::GetRobotStateStreamRequest req;
  req.set_update_rate(100.0);

  while (state_running_) {
    grpc::ClientContext ctx;
    auto stream = grpc_->state->GetRobotStateStream(&ctx, req);

    rb::api::GetRobotStateStreamResponse resp;
    while (state_running_ && stream->Read(&resp)) {
      const auto & rs = resp.robot_state();
      std::lock_guard<std::mutex> lock(state_mutex_);
      const size_t np = std::min(static_cast<size_t>(rs.position_size()), joint_positions_.size());
      for (size_t i = 0; i < np; ++i) joint_positions_[i] = rs.position(i);
      const size_t nv = std::min(static_cast<size_t>(rs.velocity_size()), joint_velocities_.size());
      for (size_t i = 0; i < nv; ++i) joint_velocities_[i] = rs.velocity(i);
      state_received_ = true;
    }

    if (state_running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
}

// ---------------------------------------------------------------------------
// Build and send a ComponentBasedCommand over the gRPC stream
// ---------------------------------------------------------------------------
void RBY1HardwareInterface::sendWriteCommand()
{
  double vx, vy, omega;
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vx    = cmd_vx_;
    vy    = cmd_vy_;
    omega = cmd_omega_;
  }

  rb::api::RobotCommandRequest req;
  req.set_priority(1);

  auto * robot_cmd = req.mutable_robot_command();
  auto * comp      = robot_cmd->mutable_component_based_command();

  // ---- Mobility: SE2VelocityCommand ----
  {
    auto * mob = comp->mutable_mobility_command();
    auto * se2 = mob->mutable_se2_velocity_command();
    *se2->mutable_command_header()->mutable_control_hold_time() = secToDuration(0.3);
    *se2->mutable_minimum_time() = secToDuration(0.1);
    auto * v = se2->mutable_velocity();
    v->mutable_linear()->set_x(vx);
    v->mutable_linear()->set_y(vy);
    v->set_angular(omega);
  }

  // ---- Body: BodyComponentBasedCommand ----
  {
    auto * body      = comp->mutable_body_command();
    auto * body_comp = body->mutable_body_component_based_command();

    // Joint values extracted by name (same order as URDF ros2_control block)
    // [0]=right_wheel [1]=left_wheel [2-7]=torso_0~5 [8-14]=right_arm_0~6
    // [15-21]=left_arm_0~6 [22-23]=head_0~1
    const auto & jn = info_.joints;
    auto findCmd = [&](const std::string & name) -> double {
      for (size_t i = 0; i < jn.size(); ++i) {
        if (jn[i].name == name) return joint_commands_[i];
      }
      return 0.0;
    };

    // Torso
    {
      auto * tc  = body_comp->mutable_torso_command();
      auto * jp  = tc->mutable_joint_position_command();
      *jp->mutable_command_header()->mutable_control_hold_time() = secToDuration(0.3);
      *jp->mutable_minimum_time() = secToDuration(0.03);
      for (int i = 0; i < 6; ++i) {
        jp->add_position(findCmd("torso_" + std::to_string(i)));
      }
    }

    // Right arm
    {
      auto * rc = body_comp->mutable_right_arm_command();
      auto * jp = rc->mutable_joint_position_command();
      *jp->mutable_command_header()->mutable_control_hold_time() = secToDuration(0.3);
      *jp->mutable_minimum_time() = secToDuration(0.03);
      for (int i = 0; i < 7; ++i) {
        jp->add_position(findCmd("right_arm_" + std::to_string(i)));
      }
    }

    // Left arm
    {
      auto * lc = body_comp->mutable_left_arm_command();
      auto * jp = lc->mutable_joint_position_command();
      *jp->mutable_command_header()->mutable_control_hold_time() = secToDuration(0.3);
      *jp->mutable_minimum_time() = secToDuration(0.03);
      for (int i = 0; i < 7; ++i) {
        jp->add_position(findCmd("left_arm_" + std::to_string(i)));
      }
    }
  }

  // ---- Head: JointPositionCommand ----
  {
    auto * head = comp->mutable_head_command();
    auto * jp   = head->mutable_joint_position_command();
    *jp->mutable_command_header()->mutable_control_hold_time() = secToDuration(0.3);
    *jp->mutable_minimum_time() = secToDuration(0.03);
    const auto & jn = info_.joints;
    for (size_t i = 0; i < jn.size(); ++i) {
      if (jn[i].name == "head_0") jp->add_position(joint_commands_[i]);
    }
    for (size_t i = 0; i < jn.size(); ++i) {
      if (jn[i].name == "head_1") jp->add_position(joint_commands_[i]);
    }
  }

  grpc_->cmd_stream->Write(req);

  // Non-blocking drain of response (we don't block the control loop)
  rb::api::RobotCommandResponse resp;
  (void)resp;
}

// ---------------------------------------------------------------------------
// gRPC helpers
// ---------------------------------------------------------------------------
bool RBY1HardwareInterface::sendPowerOn(const std::string & name)
{
  rb::api::PowerCommandRequest req;
  req.set_name(name);
  req.set_command(rb::api::PowerCommandRequest::COMMAND_POWER_ON);

  grpc::ClientContext ctx;
  rb::api::PowerCommandResponse resp;
  auto s = grpc_->power->PowerCommand(&ctx, req, &resp);
  if (!s.ok()) {
    RCLCPP_WARN(node_->get_logger(), "PowerOn RPC failed: %s", s.error_message().c_str());
    return false;
  }
  return resp.status() == rb::api::PowerCommandResponse::STATUS_SUCCESS;
}

bool RBY1HardwareInterface::sendServoOn(const std::string & name)
{
  rb::api::JointCommandRequest req;
  req.set_name(name);
  req.set_command(rb::api::JointCommandRequest::COMMAND_SERVO_ON);

  grpc::ClientContext ctx;
  rb::api::JointCommandResponse resp;
  auto s = grpc_->power->JointCommand(&ctx, req, &resp);
  if (!s.ok()) {
    RCLCPP_WARN(node_->get_logger(), "ServoOn RPC failed: %s", s.error_message().c_str());
    return false;
  }
  return resp.status() == rb::api::JointCommandResponse::STATUS_SUCCESS;
}

bool RBY1HardwareInterface::sendControlManagerCommand(int command)
{
  rb::api::ControlManagerCommandRequest req;
  req.set_command(static_cast<rb::api::ControlManagerCommandRequest::Command>(command));

  grpc::ClientContext ctx;
  rb::api::ControlManagerCommandResponse resp;
  auto s = grpc_->ctrl_mgr->ControlManagerCommand(&ctx, req, &resp);
  if (!s.ok()) {
    RCLCPP_WARN(node_->get_logger(), "ControlManagerCommand(%d) RPC failed: %s",
                command, s.error_message().c_str());
    return false;
  }
  return true;
}

}  // namespace rby1

PLUGINLIB_EXPORT_CLASS(rby1::RBY1HardwareInterface, hardware_interface::SystemInterface)
