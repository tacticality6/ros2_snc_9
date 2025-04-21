import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from ros2_snc_9_interfaces.srv import State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
import tf2_ros
import numpy as np
import time
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

class PositionTrackingNode(Node):
    def __init__(self):
        super().__init__('position_tracking_node')

        # Parameters (can be configured in the launch file)
        self.tracking_interval = self.declare_parameter('tracking_interval', 1.0).get_parameter_value().double_value  # seconds
        self.path_history_topic = self.declare_parameter('path_history_topic', 'robot_path_explore').get_parameter_value().string_value
        self.return_path_topic = self.declare_parameter('return_path_topic', 'robot_path_return').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.follow_waypoints_action_name = self.declare_parameter('follow_waypoints_action', 'follow_waypoints').get_parameter_value().string_value

        # Publishers
        self.path_publisher = self.create_publisher(Path, self.path_history_topic, 10)
        self.return_path_publisher = self.create_publisher(Path, self.return_path_topic, 10)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal variables
        self.explored_path = []  # List to store PoseStamped messages of the explored path
        self.is_exploring = True
        self.path_tracking_timer = self.create_timer(self.tracking_interval, self.track_position)

        # Nav2 Action Client for Waypoint Following
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, self.follow_waypoints_action_name)
        self.is_returning_home = False

        self.state_subscriber = self.create_subscription(
            String,
            '/snc_state',
            self.state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.get_logger().info('Position tracking node initialized (using Nav2 Waypoint Following).')
    
    def state_callback(self, msg):
        if msg.data == "Returning Home":
            self.start_return_home()

    def track_position(self):
        if self.is_exploring:
            try:
                now = self.get_clock().now()
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    now
                )

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = now.to_msg()
                pose_stamped.header.frame_id = self.map_frame
                pose_stamped.pose.position.x = transform.transform.translation.x
                pose_stamped.pose.position.y = transform.transform.translation.y
                pose_stamped.pose.position.z = transform.transform.translation.z
                pose_stamped.pose.orientation = transform.transform.rotation

                self.explored_path.append(pose_stamped)

                # Publish the current path
                path_msg = Path()
                path_msg.header.stamp = now.to_msg()
                path_msg.header.frame_id = self.map_frame
                path_msg.poses = list(self.explored_path) # Create a copy to avoid potential modification issues
                self.path_publisher.publish(path_msg)

            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'Could not transform {self.base_frame} to {self.map_frame}: {ex}')

    def start_return_home(self):
        if not self.explored_path:
            self.get_logger().warn('Explored path is empty. Cannot return home.')
            return

        self.get_logger().info('Starting return to home using Nav2 Waypoint Following.')
        self.is_exploring = False
        self.is_returning_home = True

        # Reverse the explored path to go back to the start
        reversed_path = list(reversed(self.explored_path))

        # Publish the return path (for visualization)
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Return to home goal was rejected!')
            self.is_returning_home = False
            return

        self.get_logger().info('Return to home goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Return to home finished with status: {result.result.success}')
        self.is_returning_home = False
        # Optionally, stop the robot or perform other actions here

    def feedback_callback(self, feedback_msg):
        # Optionally process feedback, e.g., to track progress along the path
        # self.get_logger().info(f'Waypoints left: {len(feedback_msg.feedback.remaining_waypoints)}')
        pass

def main(args=None):
    rclpy.init(args=args)
    position_tracking_node = PositionTrackingNode()

    try:
        position_tracking_node.spin()
    except KeyboardInterrupt:
        pass

    position_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()