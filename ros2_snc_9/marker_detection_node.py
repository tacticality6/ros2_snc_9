#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_snc_9_interfaces.srv import State
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_geometry_msgs import PointStamped
from rclpy.time import Time
import math

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        #set desired number of hazards to find
        self.num_hazards = 5
        
        # Publisher for detected hazards
        self.hazard_publisher = self.create_publisher(String, '/detected_hazards', qos_profile)
        self.marker_publisher = self.create_publisher(Marker, '/hazards', qos_profile)

        # Subscriber to /objects topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.objects_callback,
            qos_profile
        )

        # Subscriber to /scan topic
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create client for state setter service
        self.state_client = self.create_client(State, '/snc_state_setter')
        # Wait for service to be available
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('State setter service not available, waiting...')
            
        self.latest_objects = []  # Store latest extracted objects
        self.published_hazards = [] # Keep track of which hazards have been published

        # Flag to track if we have successfully called the service
        self.service_called_successfully = False

    # Function to get the hazard ID from the object ID
    def decode_hazard(self, object_id):
        hazard_mapping = {
            26: 99, 29: 99,         # Object IDs 26, 29     => Start
            19: 1,  20: 1,          # Object IDs 19, 20     => Hazard 1
            11: 2,  12: 2,  30: 2,  # Object IDs 11, 12, 30 => Hazard 2
            14: 3,  17: 3,          # Object IDs 14, 17     => Hazard 3
            10: 4,  13: 4,          # Object IDs 10, 13     => Hazard 4
            15: 5,  16: 5,          # Object IDs 15, 16     => Hazard 5
            6: 6,   8: 6,           # Object IDs 6, 8       => Hazard 6
            28: 7,  27: 7,          # Object IDs 27, 28     => Hazard 7
            23: 8,  24: 8,          # Object IDs 23, 24     => Hazard 8
            18: 9,  21: 9,          # Object IDs 18, 21     => Hazard 9
            7: 10,  9: 10,          # Object IDs 7, 9       => Hazard 10
            22: 11, 25: 11,         # Object IDs 22, 25     => Hazard 11
            4: 12,  5: 12,          # Object IDs 4, 5       => Hazard 12
        }
        return hazard_mapping.get(object_id, 0)  # Returns 0 if not found
    

    def get_robot_pose(self):
        try:
            # Lookup transform from map to base_link
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            # Extract yaw from quaternion
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return None, None, None
        
    def scan_callback(self, msg):
        self.latest_scan = msg

    def calculate_hazard_position(self, relative_x, relative_y):
        if self.latest_scan is None:
            self.get_logger().warn("No laser scan data available")

        robot_x, robot_y, robot_yaw = self.get_robot_pose()
        if robot_x is None:
            self.get_logger().warn("Cannot calculate hazard position: robot pose unavailable")
            return None, None

        # Calculate bearing from relative coordinates
        bearing = math.atan2(relative_y, relative_x)
        # Convert bearing to laser scan angle (relative to robot's forward direction)
        scan_angle = bearing + robot_yaw  # Adjust for robot's orientation
        # Normalize angle to [0, 2pi]
        scan_angle = scan_angle % (2 * math.pi)
        
        # Find the corresponding laser scan index
        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        num_ranges = len(scan.ranges)
        
        # Calculate the index for the scan angle
        index = int((scan_angle - angle_min) / angle_increment)
        if index < 0 or index >= num_ranges:
            self.get_logger().warn(f"Scan angle {scan_angle} out of range")
            return None, None

        # Get the distance from the laser scan
        distance = scan.ranges[index]
        if math.isinf(distance) or math.isnan(distance):
            self.get_logger().warn(f"Invalid laser scan distance at index {index}")
            return None, None

        # Calculate hazard position in map frame
        hazard_x = robot_x + distance * math.cos(robot_yaw + bearing)
        hazard_y = robot_y + distance * math.sin(robot_yaw + bearing)

        return hazard_x, hazard_y

    # def get_robot_position(self):
    #     try:
    #         # Lookup transform from map to base_link
    #         trans = self.tf_buffer.lookup_transform(
    #             'map',
    #             'base_link',
    #             Time(),
    #             rclpy.duration.Duration(seconds=1.0)
    #         )
    #         return trans.transform.translation.x, trans.transform.translation.y
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
    #             tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().warn(f"TF lookup failed: {str(e)}")
    #         return None, None
        
    def create_hazard_marker(self, x, y, hazard_id, object_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hazards"
        marker.id = object_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color based on hazard type
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # Persistent
        return marker

    # Function to send request to set the state to "Exploring" via the service
    def set_exploring_state(self, object_id, x_position, y_position):
        # Only proceed if we haven't successfully called the service yet
        if self.service_called_successfully:
            self.get_logger().info(f"Ignoring Object ID {object_id} at ({x_position}, {y_position}) - Service already called successfully")
            return
        
        request = State.Request()
        request.new_state = "Exploring"
        
        self.get_logger().info(f"Object ID {object_id} at ({x_position}, {y_position}) triggered 'Exploring' state")
        
        # Send the request asynchronously
        future = self.state_client.call_async(request)
        future.add_done_callback(self.state_response_callback)

    # Function to handle service response
    def state_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully set state to 'Exploring'")
                # Set the flag to indicate successful service call
                self.service_called_successfully = True
            else:
                self.get_logger().warn("Failed to set state to 'Exploring'")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    # Process incoming /objects messages, extracts object data and publishes
    def objects_callback(self, msg):
        data = msg.data
        object_list = []
        hazard_messages = []
        
        if len(data) % 12 == 0:  # Each object has 12 float values
            for i in range(0, len(data), 12):
                object_id = int(data[i])
                relative_x = data[i + 9]
                relative_y = data[i + 10]
                hazard = self.decode_hazard(object_id)
                
                if hazard is not None:
                    # Calculate hazard position using laser scan
                    hazard_x, hazard_y = self.calculate_hazard_position(relative_x, relative_y)
                    if hazard_x is None or hazard_y is None:
                        self.get_logger().warn(f"Skipping object {object_id} due to position calculation failure")
                        continue

                    object_list.append((object_id, hazard_x, hazard_y, hazard))
                    
                    # If hazard ID is 99, send "Exploring" to the service
                    if hazard == 99:
                        self.set_exploring_state(object_id, hazard_x, hazard_y)
                    elif hazard not in self.published_hazards:
                        # For other hazards, add to the messages to be published
                        hazard_messages.append(f"Object ID {object_id} at ({hazard_x:.2f}, {hazard_y:.2f}) is Hazard {hazard}")
                        self.published_hazards.append(hazard)

                        # Publish marker at hazard position
                        marker = self.create_hazard_marker(hazard_x, hazard_y, hazard, object_id)
                        self.marker_publisher.publish(marker)
            
            self.latest_objects = object_list  # Store extracted data
            
            # Publish detected hazards if any non-99 hazards are found
            if hazard_messages:
                msg = String()
                msg.data = " | ".join(hazard_messages)
                self.hazard_publisher.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')
            if len(hazard_messages) == self.num_hazards:
                request = State.Request()
                request.new_state = "Returning Home"

                future = self.state_client.call_async(request)
                self.get_logger().info("Attempting to return home")
                future.add_done_callback(self.set_return_home_callback)
        else:
            self.get_logger().warn("Received unexpected /objects message format")
    
    def set_return_home_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully set state to 'Returning Home'")
                # Set the flag to indicate successful service call
                self.service_called_successfully = True
            else:
                self.get_logger().warn("Failed to set state to 'Returning Home'")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()