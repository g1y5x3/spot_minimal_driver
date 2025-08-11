import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from spot_action.action import MoveRelativeXY


class NavGoalListener(Node):
    def __init__(self):
        super().__init__("nav_goal_listener")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("robot_frame", "base_link")
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value

        self.subscription = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self._move_client = ActionClient(self, MoveRelativeXY, "move_relative_xy")

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal in frame: {msg.header.frame_id}")
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.robot_frame,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            transformed_goal = do_transform_pose(msg.pose, transform)

        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            return

        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("MoveRelativeXY action server not available!")
            return

        goal_msg = MoveRelativeXY.Goal()
        goal_msg.x = transformed_goal.position.x
        goal_msg.y = transformed_goal.position.y

        q = transformed_goal.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        goal_msg.yaw = yaw

        self.get_logger().info(f"Transformed goal:\nx: {goal_msg.x:.2f}, y: {goal_msg.y:.2f}, yaw: {goal_msg.yaw:.2f}")

        self._move_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Move goal rejected")
            return

        self.get_logger().info("Move goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"MoveRelativeXY result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
