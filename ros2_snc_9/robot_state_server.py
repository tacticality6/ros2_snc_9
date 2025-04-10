#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from ros2_snc_9_interfaces.srv import State
from rclpy.qos import QoSProfile, ReliabilityPolicy

#Node to control robots current state
class StateServer(Node):
    def __init__(self, node_name="state_server_node"):
        self._node_name = node_name
        super().__init__(self._node_name)


        self.current_state = "Awaiting Start"
        self.state_list = ['Awaiting Start', 'Exploring', 'Returning Home', 'Manual INTERVENTION']

        self.state_publisher = self.create_publisher(
            String, 
            "/snc_state",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.update_timer = self.create_timer(1, self.timer_callback) #update state topic every second

        #service for state setting
        self.state_setter_service = self.create_service(
            State,
            "/snc_state_setter",
            self.set_state_callback    
        )

        
        
        #status publisher, for contextual info as per spec
        self.status_publisher = self.create_publisher(
            String, 
            "/snc_status",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        #subscriptions to intervention topics
        self.start_intervention_sub = self.create_subscription(
            Empty,
            '/trigger_start',
            self.start_intervention_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.teleop_intervention_sub = self.create_subscription(
            Empty,
            '/trigger_teleop',
            self.teleop_intervention_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.return_intervention_sub = self.create_subscription(
            Empty,
            '/trigger_home',
            self.return_intervention_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.get_logger().info("State Manager Ready...")


    def set_state_callback(self, request, response):
        if request.new_state not in self.state_list:
            response.success = False
            return response
        self.current_state = request.new_state
        response.success = True
        return response
    

    def timer_callback(self):
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        self.get_logger().debug(f"State is: {self.current_state}")

        if self.current_state == "Awaiting Start":
            msg.data = msg.data + "..."
            self.status_publisher.publish(msg)

    
    def start_intervention_callback(self, msg):
        self.current_state = 'Exploring'
        newMsg = String()
        newMsg.data = "Start Trigger Intervention Received"
        self.get_logger().info(newMsg.data)
        self.status_publisher.publish(newMsg)
    
    def return_intervention_callback(self, msg):
        self.current_state = 'Returning Home'
        newMsg = String()
        newMsg.data = "Return Home Intervention Received"
        self.get_logger().info(newMsg.data)
        self.status_publisher.publish(newMsg)
    
    def teleop_intervention_callback(self, msg):
        self.current_state = 'Manual INTERVENTION'
        newMsg = String()
        newMsg.data = "Teleop Intervention Received"
        self.get_logger().info(newMsg.data)
        self.status_publisher.publish(newMsg)
        #TODO: Implement teleop

        
def main(args=None):
    rclpy.init(args=args)

    state_server = StateServer()

    try:
        rclpy.spin(state_server)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    state_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()