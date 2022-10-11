#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np


class VisualizationNode(Node):
    def __init__(self):
        super().__init__("vis_node")
        plt.ion()

        self.figure, self.ax = plt.subplots(1, 1,
                                figsize =(5, 5),
                                tight_layout = True)
        plt.axis('equal')
        self.x = []
        self.y = []
        self.weights = []
        
        self.create_subscription(Float32MultiArray, "particle_weights", self.process_weights, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "particle_x", self.process_x, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "particle_y", self.process_y, qos_profile_sensor_data)
        self.create_timer(5, self.plot_particles_with_weights)
    
    def plot_particles_with_weights(self):
        if len(self.x) == 0:
            return
        print(self.x, self.y, self.weights)
        
        self.scat = self.ax.scatter(self.x, self.y, s=np.array(self.weights)*500)
        plt.xlim([-1.3, 2.3])
        plt.ylim([-3, 0.8])
        
        plt.show()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.cla()

    def process_x(self, x):
        self.x = x.data

    def process_y(self, y):
        self.y = y.data

    def process_weights(self, weights):
        self.weights = weights.data
    
    def plot_particle_weights(self, weights):
        if len(set(weights.data)) <= 1:
            return
        
        self.hist, _, _ = self.ax.hist(weights.data, bins = 100)
        
        plt.xlim([0, 0.2])
        # plt.ylim([0, 300])
        plt.show()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.cla()
        


def main(args=None):
    rclpy.init(args=args)

    vis_node = VisualizationNode()
    rclpy.spin(vis_node)
    # ani = FuncAnimation(vis_node.fig, vis_node.plot_particle_weights, init_func=vis_node.plot_init)

    vis_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()