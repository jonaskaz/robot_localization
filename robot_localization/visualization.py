#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class VisualizationNode(Node):
    def __init__(self):
        super().__init__("vis_node")
        self.figure, self.ax = plt.subplots(figsize=(10, 8))
        self.hist = plt.hist([], bins = 20)
        self.w_data = []
        self.create_subscription(Float32MultiArray, "particle_weights", self.plot_particle_weights, qos_profile_sensor_data)
    
    def plot_particle_weights(self, weights):
        print(weights.data)
        # self.w_data = weights
        # plt.hist(weights.data)

def main(args=None):
    rclpy.init(args=args)

    vis_node = VisualizationNode()
    rclpy.spin(vis_node)

    # ani = FuncAnimation(vis_node.fig, vis_node.update_plot, init_func=vis_node.plot_init)
    # plt.show(block=True) 

    vis_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()