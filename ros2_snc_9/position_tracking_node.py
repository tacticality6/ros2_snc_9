import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
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
import math
from enum import Enum

class RobotState(Enum):
    EXPLORING = 1
    RETURNING_HOME = 2
    IDLE = 3

class PositionTrackingNode(Node):
    def __init__(self):
        super().__init__('position_tracking_node')

        # Parameters
        self.tracking_interval = self.declare_parameter('tracking_interval', 1.0).get_parameter_value().double_value
        self.path_history_topic = self.declare_parameter('path_history_topic', 'path_explore').get_parameter_value().string_value
        self.return_path_topic = self.declare_parameter('return_path_topic', 'path_return').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.follow_waypoints_action_name = self.declare_parameter('follow_waypoints_action', 'follow_waypoints').get_parameter_value().string_value
        self.path_history_max_length = self.declare_parameter('path_history_max_length', 1000).get_parameter_value().integer_value  # Maximum path length
        #self.rotation_before_return = self.declare_parameter('rotation_before_return', True).get_parameter_value().bool_value # Rotate 180 before returning

        # Publishers
        self.path_publisher = self.create_publisher(Path, self.path_history_topic, qos_profile_sensor_data)
        self.return_path_publisher = self.create_publisher(Path, self.return_path_topic, qos_profile_sensor_data)
        #self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # For manual rotation

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
                buffer_time = Duration(seconds=0.1)
                safe_time = now - buffer_time
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    safe_time,
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

        #reverse rotation in each pose
        for i, pose in enumerate(reversed_path):
            new_pos = pose.position
            new_orientation = pose.orientation
            roll, pitch, yaw = euler_from_quaternion(new_orientation.x,new_orientation.y,new_orientation.z,new_orientation.w)
            new_yaw = -yaw

            new_orientation = quaternion_from_euler(roll, pitch, new_yaw)

            new_pose = PoseStamped()
            new_pose.header = pose.header
            new_pose.pose.position = new_pos
            
            new_pose.pose.orientation.x = new_orientation[0]
            new_pose.pose.orientation.y = new_orientation[1]
            new_pose.pose.orientation.z = new_orientation[2]
            new_pose.pose.orientation.w = new_orientation[3]

            reversed_path[i] = new_pose


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

    try:
        rclpy.spin(position_tracking_node)
    except KeyboardInterrupt:
        pass

    position_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def euler_from_quaternion(x,y,z,w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q