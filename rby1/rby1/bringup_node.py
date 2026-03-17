#!/usr/bin/env python3
import threading

import grpc
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rb.api import power_pb2
from rb.api import power_service_pb2_grpc
from rb.api import robot_info_pb2
from rb.api import robot_info_service_pb2_grpc
from rb.api import robot_state_pb2  # GetRobotStateRequest, GetRobotStateStreamRequest, PowerState
from rb.api import robot_state_service_pb2_grpc


class BringupNode(Node):
    def __init__(self):
        super().__init__('rby1_bringup_node')

        self.declare_parameter('robot_ip', '192.168.12.1:5173')
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.get_logger().info(f'Connecting to robot at {robot_ip}')
        self._channel = grpc.insecure_channel(robot_ip)

        self._power_stub = power_service_pb2_grpc.PowerServiceStub(self._channel)
        self._state_stub = robot_state_service_pb2_grpc.RobotStateServiceStub(self._channel)
        self._info_stub = robot_info_service_pb2_grpc.RobotInfoServiceStub(self._channel)

        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._joint_names = self._get_joint_names()
        self._ensure_power_on()
        self._ensure_servo_on()

        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    def _get_joint_names(self):
        self.get_logger().info('Getting robot info...')
        req = robot_info_pb2.GetRobotInfoRequest()  # defined in robot_info_pb2
        resp = self._info_stub.GetRobotInfo(req)
        names = [ji.name for ji in resp.robot_info.joint_infos]
        self.get_logger().info(f'Found {len(names)} joints: {names}')
        return names

    def _get_state(self):
        req = robot_state_pb2.GetRobotStateRequest()
        resp = self._state_stub.GetRobotState(req)
        return resp.robot_state

    def _ensure_power_on(self):
        state = self._get_state()
        is_power_on = any(
            ps.state == robot_state_pb2.PowerState.STATE_POWER_ON
            for ps in state.power_states
        )
        if not is_power_on:
            self.get_logger().info('Power is off. Turning power on...')
            req = power_pb2.PowerCommandRequest(
                name='.*',
                command=power_pb2.PowerCommandRequest.COMMAND_POWER_ON,
            )
            self._power_stub.PowerCommand(req)
            self.get_logger().info('Power on complete.')
        else:
            self.get_logger().info('Power already on.')

    def _ensure_servo_on(self):
        state = self._get_state()
        is_servo_on = (
            len(state.joint_states) > 0
            and all(js.is_ready for js in state.joint_states)
        )
        if not is_servo_on:
            self.get_logger().info('Servo is off. Turning servo on...')
            req = power_pb2.JointCommandRequest(
                name='.*',
                command=power_pb2.JointCommandRequest.COMMAND_SERVO_ON,
            )
            self._power_stub.JointCommand(req)
            self.get_logger().info('Servo on complete.')
        else:
            self.get_logger().info('Servo already on.')

    def _stream_loop(self):
        req = robot_state_pb2.GetRobotStateStreamRequest(update_rate=100.0)
        try:
            for resp in self._state_stub.GetRobotStateStream(req):
                self._publish_joint_states(resp.robot_state)
        except grpc.RpcError as e:
            self.get_logger().error(f'State stream error: {e}')

    def _publish_joint_states(self, robot_state):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names

        js = robot_state.joint_states
        msg.position = [s.position for s in js]
        msg.velocity = [s.velocity for s in js]
        msg.effort = [s.torque for s in js]

        self._joint_state_pub.publish(msg)

    def destroy_node(self):
        self._channel.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BringupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
