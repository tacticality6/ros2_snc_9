#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty
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

        #reset map on load
        self.client = self.create_client(Empty, '/slam_toolbox/clear')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /slam_toolbox/clear service...')

        self.send_clear_request()
        self.get_logger().info("Navigation Logic Up. Awaiting Start...")

    def send_clear_request(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.clear_response_callback)

    def clear_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Successfully reset SLAM map')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def state_callback(self, msg):
        if msg.data == "Exploring":
            self.exploring = True
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