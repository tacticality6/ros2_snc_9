import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from ros2_snc_9_interfaces.srv import State
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
import tf2_ros
import numpy as np
import time
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from enum import Enum

class RobotState(Enum):
    EXPLORING = 1
    RETURNING_HOME = 2
    IDLE = 3

class PositionTrackingNode(Node):
    def __init__(self):
        super().__init__('position_tracking_node')

        # Parameters
        self.tracking_interval = self.declare_parameter('tracking_interval', 1.0).get_parameter_value().double_value  # seconds
        self.path_history_topic = self.declare_parameter('path_history_topic', 'robot_path_explore').get_parameter_value().string_value
        self.return_path_topic = self.declare_parameter('return_path_topic', 'robot_path_return').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.follow_waypoints_action_name = self.declare_parameter('follow_waypoints_action', 'follow_waypoints').get_parameter_value().string_value
        self.path_history_max_length = self.declare_parameter('path_history_max_length', 1000).get_parameter_value().integer_value  # Maximum path length

        # Publishers
        self.path_publisher = self.create_publisher(Path, self.path_history_topic, qos_profile_sensor_data)
        self.return_path_publisher = self.create_publisher(Path, self.return_path_topic, qos_profile_sensor_data)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal variables
        self.explored_path = []  # List to store PoseStamped messages of the explored path
        self.robot_state = RobotState.EXPLORING
        self.path_tracking_timer = self.create_timer(self.tracking_interval, self.track_position)

        # Nav2 Action Client for Waypoint Following
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, self.follow_waypoints_action_name)
        self._goal_handle = None

        # Subscriber for state changes
        self.state_subscriber = self.create_subscription(
            String,
            '/snc_state',
            self.state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.get_logger().info('Position tracking node initialized (using Nav2 Waypoint Following).')

    def state_callback(self, msg):
        if msg.data == "Returning Home" and self.robot_state == RobotState.EXPLORING:
            self.start_return_home()
        elif msg.data == "Exploring" and self.robot_state == RobotState.RETURNING_HOME:
            self.get_logger().info('Received "Exploring" state while returning home. Ignoring.')
        elif msg.data == "Idle" and self.robot_state == RobotState.RETURNING_HOME:
            self.get_logger().info('Received "Idle" state. Assuming return home is complete.')
            self.robot_state = RobotState.IDLE

    def track_position(self):
        if self.robot_state == RobotState.EXPLORING:
            try:
                now = self.get_clock().now()
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = now.to_msg()
                pose_stamped.header.frame_id = self.map_frame
                pose_stamped.pose.position.x = transform.transform.translation.x
                pose_stamped.pose.position.y = transform.transform.translation.y
                pose_stamped.pose.position.z = transform.transform.translation.z
                pose_stamped.pose.orientation = transform.transform.rotation

                self.explored_path.append(pose_stamped)

                # Limit the length of the explored path
                if len(self.explored_path) > self.path_history_max_length:
                    self.explored_path.pop(0)

                # Publish the current path
                path_msg = Path()
                path_msg.header.stamp = now.to_msg()
                path_msg.header.frame_id = self.map_frame
                path_msg.poses = list(self.explored_path)
                self.path_publisher.publish(path_msg)

            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'Could not transform {self.base_frame} to {self.map_frame}: {ex}')

    def start_return_home(self):
        if not self.explored_path:
            self.get_logger().warn('Explored path is empty. Cannot return home.')
            return

        if self.robot_state == RobotState.RETURNING_HOME:
            self.get_logger().info('Return home already in progress.')
            return

        self.get_logger().info('Starting return to home using Nav2 Waypoint Following.')
        self.robot_state = RobotState.RETURNING_HOME

        # Reverse the explored path to go back to the start
        reversed_path = list(reversed(self.explored_path))

        # Publish the return path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        path_msg.poses = reversed_path
        self.return_path_publisher.publish(path_msg)

        # Send the reversed path as a FollowWaypoints goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = reversed_path

        self._send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Return to home goal was rejected!')
            self.robot_state = RobotState.EXPLORING # Revert state on rejection
            self._goal_handle = None
            return

        self.get_logger().info('Return to home goal accepted.')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info(f'Return to home finished with status: {result.result.success}')
            self.robot_state = RobotState.IDLE
        else:
            self.get_logger().error('Return to home failed to get result.')
            self.robot_state = RobotState.EXPLORING # Revert state on failure
        self._goal_handle = None

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    position_tracking_node = PositionTrackingNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(position_tracking_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        position_tracking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()