#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



class VisualizationNode(Node):
    def __init__(self):
        super().__init__("vis_node")
        self.vis_pub = self.create_subscription(Float32MultiArray, "particle_weights", 10)

    def received_weights(self, weights):
        print(weights)


def main(args=None):
    rclpy.init(args=args)

    vis_node = VisualizationNode()

    rclpy.spin(vis_node)

    vis_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()