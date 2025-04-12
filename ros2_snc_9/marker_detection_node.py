#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_snc_9_interfaces.srv import State

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Publisher for detected hazards
        self.hazard_publisher = self.create_publisher(String, '/detected_hazards', qos_profile)
        
        # Subscriber to /objects topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.objects_callback,
            qos_profile
        )
        
        # Create client for state setter service
        self.state_client = self.create_client(State, '/snc_state_setter')
        # Wait for service to be available
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('State setter service not available, waiting...')
            
        self.latest_objects = []  # Store latest extracted objects

        # Flag to track if we have successfully called the service
        self.service_called_successfully = False

    # Function to get the hazard ID from the object ID
    def decode_hazard(self, object_id):
        hazard_mapping = {
            26: 99, 29: 99,         # Object IDs 26, 29     => Start
            11: 2, 12: 2, 30: 2,    # Object IDs 11, 12, 30 => Hazard 2
        }
        return hazard_mapping.get(object_id, None)  # Returns None if not found

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
                x_position = data[i + 1]
                y_position = data[i + 2]
                hazard = self.decode_hazard(object_id)
                
                if hazard is not None:
                    object_list.append((object_id, x_position, y_position, hazard))
                    
                    # If hazard ID is 99, send "Exploring" to the service instead of publishing
                    if hazard == 99:
                        self.set_exploring_state(object_id, x_position, y_position)
                    else:
                        # For other hazards, add to the messages to be published
                        hazard_messages.append(f"Object ID {object_id} at ({x_position}, {y_position}) is Hazard {hazard}")
            
            self.latest_objects = object_list  # Store extracted data
            
            # Publish detected hazards if any non-99 hazards are found
            if hazard_messages:
                msg = String()
                msg.data = " | ".join(hazard_messages)
                self.hazard_publisher.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')
        else:
            self.get_logger().warn("Received unexpected /objects message format")

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