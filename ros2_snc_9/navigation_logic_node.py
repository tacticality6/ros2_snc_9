#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs import String
from std_msgs import Bool
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
    
    def state_callback(self, msg):
        if msg.data == "Exploring":
            self.exploring = True
        else:
            self.exploring = False
    
    def timer_callback(self):
        
        if self.exploring:
            self.get_logger().info("Exploring...")
            if not self.exploreLiteActive:
                self.get_logger().info("Resuming Exploration...")
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
