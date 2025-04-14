#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class NavigationNode(Node):
    def __init__(self, node_name):
        self._node_name = node_name
        super().__init__(node_name)

        self.exploring = False
        self.exploreLiteActive = False

        self.update_timer = self.create_timer(1, self.timer_callback)

        self.state_subscriber = self.create_subscription(
            String,
            '/snc_state',
            self.state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.status_publisher = self.create_publisher(
            String,
            '/snc_status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.explore_toggle_pub = self.create_publisher(
            Bool,
            'explore/resume',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.motion_pub = self.create_publisher(
            Twist,
            'cmd_vel/',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )        

        self.get_logger().info("Navigation Logic Up. Awaiting Start...")
    
    def state_callback(self, msg):
        if msg.data == "Exploring":
            self.exploring = True
            self.doInitialMovement()
        else:
            self.exploring = False
    
    def timer_callback(self):
        
        if self.exploring:
            if not self.exploreLiteActive:
                self.get_logger().info("Exploring...")
                self.exploreLiteActive = True
                msg = Bool()
                msg.data = self.exploreLiteActive
                self.explore_toggle_pub.publish(msg)

        else:
            if self.exploreLiteActive:
                self.get_logger().info("Pausing Exploration...")
                self.exploreLiteActive = False
                msg = Bool()
                msg.data = self.exploreLiteActive
                self.explore_toggle_pub.publish(msg)
    
    def doInitialMovement(self):
        initialMove = Twist()
        initialMove.angular.z = 0.5

        msg = String()
        msg.data = 'Sending initial motion command...'
        self.get_logger().info(msg.data)
        self.status_publisher().publish(msg)
        self.motion_pub.publish(initialMove)

        self.stop_timer = self.create_timer(1.0, self.stop_initial)

    def stop_initial(self):
        #Stop the robot
        self.stop_timer.cancel()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.motion_pub.publish(twist)

        msg = String()
        msg.data = 'Stopping initial motion'
        self.get_logger().info(msg.data)
        self.status_publisher().publish(msg)



def main(args=None):
    rclpy.init(args=args)

    nav_node = NavigationNode("navigation_logic_node")

    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()