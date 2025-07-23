# Copyright 2025 Yixiang Gao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A minimal ROS 2 driver for Boston Dynamics Spot robot."""

import threading
import time
from typing import Optional

import bosdyn.client
import bosdyn.client.util
from bosdyn.api.geometry_pb2 import SE3Pose
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.lease import Error as LeaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient, RobotStateStreamingClient

from geometry_msgs.msg import TransformStamped, Twist

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster


class SpotROS2Driver(Node):
    """A minimal ROS 2 driver for Boston Dynamics Spot robot."""

    def __init__(self):
        """Initialize the Spot ROS 2 driver node."""
        super().__init__('spot_driver_node')

        self.declare_parameter('hostname', '192.168.80.3')
        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        # TODO: Add parameter for robot username and password if needed

        self._latest_state = None
        self._state_lock = threading.Lock()

        self.robot: Optional[bosdyn.client.robot.Robot] = None
        self.lease_keep_alive: Optional[LeaseKeepAlive] = None
        self.estop_keep_alive: Optional[EstopKeepAlive] = None
        self.robot_state_client: Optional[RobotStateClient] = None
        self.command_client: Optional[RobotCommandClient] = None

        try:
            # Robot initialization
            sdk = bosdyn.client.create_standard_sdk('SpotROS2DriverClient')
            # Register the non-standard api clients
            # https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/noarm_squat.py
            sdk.register_service_client(RobotStateStreamingClient)

            self.robot = sdk.create_robot(self.hostname)
            bosdyn.client.util.authenticate(self.robot)
            self.robot.time_sync.wait_for_sync()

            # NOTE: Not sure if this is necessary
            assert not self.robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                                 'such as the estop SDK example, to configure E-Stop.'

            self.get_logger().info('Successfully authenticated and connected to the robot.')

            # Create SDK clients
            self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
            self.robot_state_streaming_client = self.robot.ensure_client(RobotStateStreamingClient.default_service_name)
            self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
            estop_client = self.robot.ensure_client(EstopClient.default_service_name)
            self.get_logger().info('Robot clients created.')

            # Lease management
            self.lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
            self.get_logger().info('Acquired lease.')

            # Acquire E-Stop
            estop_endpoint = EstopEndpoint(estop_client, 'SpotROS2DriverEStop', 10.0)
            estop_endpoint.force_simple_setup()
            self.estop_keep_alive = EstopKeepAlive(estop_endpoint)
            self.get_logger().info('Acquired E-Stop.')

            time.sleep(2.0)

            # Power on and Stand Robot
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), 'Robot power on failed.'
            self.get_logger().info('Robot powered on.')

            blocking_stand(self.command_client, timeout_sec=10)
            self.get_logger().info('Robot standing.')

        except (RpcError, ResponseError, LeaseError) as e:
            self.get_logger().error(f'Failed to connect to the robot: {e}')
            raise

        # ROS 2 publishers and subscribers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Start the background thread for state streaming
        self._state_thread = threading.Thread(target=self._state_streaming_thread, daemon=True)
        self._state_thread.start()

        # Main Loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    def _state_streaming_thread(self):
        """Continuously gets robot state and caches it thread-safely."""
        try:
            for robot_state in self.robot_state_streaming_client.get_robot_state_stream():
                with self._state_lock:
                    self.get_logger().info(f"Received object of type: {type(robot_state)}")
                    self._latest_state = robot_state
        except Exception as e:
            self.get_logger().error(f'Error in state streaming thread: {e}')

    def timer_callback(self):
        """Periodic publish robot data (if connected)."""
        with self._state_lock:
            robot_state = self._latest_state

        if robot_state:
            odom_tfrom_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
            self.publish_transform(odom_tfrom_body)
        # robot_state = self.robot_state_client.get_robot_state()
        # odom_tfrom_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
        # self.publish_transform(odom_tfrom_body)

    def publish_transform(self, odom_tfrom_body: SE3Pose):  # type: ignore
        """Publish the transform from ODOM to BODY frame."""
        t = TransformStamped()
        # TODO: sync with the robot's internal time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_tfrom_body.position.x
        t.transform.translation.y = odom_tfrom_body.position.y
        t.transform.translation.z = odom_tfrom_body.position.z
        t.transform.rotation.x = odom_tfrom_body.rotation.x
        t.transform.rotation.y = odom_tfrom_body.rotation.y
        t.transform.rotation.z = odom_tfrom_body.rotation.z
        t.transform.rotation.w = odom_tfrom_body.rotation.w

        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        """Convert a Twist message to a robot velocity command and send it."""
        v_x, v_y, v_rot = msg.linear.x, msg.linear.y, msg.angular.z

        command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot)
        end_time = time.time() + 0.5

        try:
            # Send the command to the robot
            self.command_client.robot_command(command, end_time_secs=end_time)
            self.get_logger().debug(f'Sent velocity command: v_x={v_x}, v_y={v_y}, v_rot={v_rot}')
        except (RpcError, ResponseError) as e:
            self.get_logger().error(f'Failed to send velocity command: {e}')

    def shutdown(self):
        """Shutdown the driver and release resources."""
        # power off requires lease so we do it before releasing
        if self.robot and self.robot.is_powered_on():
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            print('Robot powered off.')

        # Release the E-Stop.
        if self.estop_keep_alive:
            self.estop_keep_alive.shutdown()
            print('E-Stop released.')

        if self.lease_keep_alive:
            self.lease_keep_alive.shutdown()
            print('Lease released.')


def main(args=None):
    """Initialize and run the Spot ROS 2 driver node."""
    rclpy.init(args=args)
    spot_driver_node = None

    try:
        spot_driver_node = SpotROS2Driver()
        if rclpy.ok():
            rclpy.spin(spot_driver_node)
    except KeyboardInterrupt:
        if spot_driver_node:
            print('Shutting down the Robot due to KeyboardInterrupt.')
    except (RpcError, ResponseError, LeaseError) as e:
        if spot_driver_node:
            print(f'Shutting down the Robot due to Spot-SDK error: {e}')
    finally:
        if spot_driver_node:
            spot_driver_node.shutdown()
            spot_driver_node.destroy_node()


if __name__ == '__main__':
    main()
