#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
)
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import (
    TFHelper,
    draw_random_sample,
    gaussian,
    calculate_mean_angle,
)
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=0.0):
        """Construct a new Particle
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized"""
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """A helper function to convert a particle to a geometry_msgs/Pose message"""
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )


class ParticleFilter(Node):
    """The class that represents a Particle Filter ROS Node
    Attributes list:
        base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
        map_frame: the name of the map coordinate frame (should be "map" in most cases)
        odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
        scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
        n_particles: the number of particles in the filter
        d_thresh: the amount of linear movement before triggering a filter update
        a_thresh: the amount of angular movement before triggering a filter update
        pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        particle_pub: a publisher for the particle cloud
        last_scan_timestamp: this is used to keep track of the clock when using bags
        scan_to_process: the scan that our run_loop should process next
        occupancy_field: this helper class allows you to query the map for distance to closest obstacle
        transform_helper: this helps with various transform operations (abstracting away the tf2 module)
        particle_cloud: a list of particles representing a probability distribution over robot poses
        current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                               The pose is expressed as a list [x,y,theta] (where theta is the yaw)
        thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__("pf")
        self.base_frame = "base_footprint"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from

        self.n_particles = 300  # the number of particles to use

        self.initial_particle_spread = 3  # spread of initial particles

        # the amount of linear movement before performing an update
        self.d_thresh = 0.2
        # the amount of angular movement before performing an update
        self.a_thresh = math.pi / 6

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.update_initial_pose, 10
        )

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            PoseArray, "particlecloud", qos_profile_sensor_data
        )

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()

        # we are using a thread to work around single threaded execution bottleneck
        thread_2 = Thread(target=self.pub_wrapper)
        thread_2.start()

        # Publish particle weights for visualizing
        self.particle_weight_publisher = self.create_publisher(
            Float32MultiArray, "particle_weights", qos_profile_sensor_data
        )
        self.particle_x_publisher = self.create_publisher(
            Float32MultiArray, "particle_x", qos_profile_sensor_data
        )
        self.particle_y_publisher = self.create_publisher(
            Float32MultiArray, "particle_y", qos_profile_sensor_data
        )

    def pub_latest_transform(self):
        """This function takes care of sending out the map to odom transform"""
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(
            seconds=0.1
        )
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp
        )

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded exe cutors in ROS2"""
        while True:
            self.run_loop()
            time.sleep(0.1)
    
    def pub_wrapper(self):
        while True:
            self.pub_latest_transform()
            time.sleep(0.05)

    def run_loop(self):
        """This is the main run_loop of our particle filter.  It checks to see if
        any scans are ready and to be processed and will call several helper
        functions to complete the processing.

        You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(
            self.odom_frame, self.base_frame, msg.header.stamp
        )
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return

        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame
        )
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()  # update based on odometry
            # updates weight based on laser scan
            self.update_particles_with_laser(r, theta)
            self.update_robot_pose()  # update robot's pose based on particles
            # resample particles to focus on areas of high density
            self.resample_particles()
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return (
            math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            > self.a_thresh
        )

    def update_robot_pose(self):
        """Update the estimate of the robot's pose given the updated particles.
        There are two logical methods for this:
            (1): compute the mean pose
            (2): compute the most likely pose (i.e. the mode of the distribution)

        Note: we decided to use the mean pose approach since it prevents the
        pose from jumping around as much
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # Compute the mean pose by averaging all particle considering weights
        x_mean = sum(p.x * p.w for p in self.particle_cloud)
        y_mean = sum(p.y * p.w for p in self.particle_cloud)
        # Calculate the mean angle by converting the computing unit vectors
        # from the list of angles and adding the weighted vectors
        theta_mean = calculate_mean_angle(
            [p.theta for p in self.particle_cloud], [p.w for p in self.particle_cloud]
        )
        mean_pose = Particle(x_mean, y_mean, theta_mean)
        self.robot_pose = mean_pose.as_pose()
        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, self.odom_pose)

    def update_particles_with_odom(self):
        """Update the particles using the newly given odometry pose.
        The function computes the value delta which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """

        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )

        # Compute the transformation matrix between the last pose and the new pose.
        if self.current_odom_xy_theta:

            # Let t1 represent the initial position and t2 be the updated position after moving
            # Transformation matrix from the t1 robot pose to the odom frame
            c, s = np.cos(self.current_odom_xy_theta[2]), np.sin(
                self.current_odom_xy_theta[2]
            )
            T_t1_to_odom = np.array(
                (
                    (c, -s, self.current_odom_xy_theta[0]),
                    (s, c, self.current_odom_xy_theta[1]),
                    (0, 0, 1),
                )
            )
            # Transformation matrix from the t2 robot pose to the odom frame
            c, s = np.cos(new_odom_xy_theta[2]), np.sin(new_odom_xy_theta[2])
            T_t2_to_odom = np.array(
                ((c, -s, new_odom_xy_theta[0]), (s, c, new_odom_xy_theta[1]), (0, 0, 1))
            )
            # Since we have transformation matrices for t1->odom and t2->odom,
            # we can calculate t2->t1, which puts t2 in terms of t1 and eliminates
            # the odom frame. We do this by inversing t1->odom and matrix multiplying
            # it with t2->odom.
            T_t2_to_t1 = np.linalg.inv(T_t1_to_odom) @ T_t2_to_odom

            # Update the current position
            self.current_odom_xy_theta = new_odom_xy_theta

        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # Apply transformation to the particles
        for particle in self.particle_cloud:
            # Create matrix for the particle so we can apply the transformation
            c, s = np.cos(particle.theta), np.sin(particle.theta)
            T_particle = np.array(((c, -s, particle.x), (s, c, particle.y), (0, 0, 1)))
            # Apply particle transformation
            new_particle_loc = T_particle @ T_t2_to_t1
            particle.x = new_particle_loc[0][2]
            particle.y = new_particle_loc[1][2]
            particle.theta = np.arctan2(new_particle_loc[1][0], new_particle_loc[0][0])

    def resample_particles(self):
        """Resample particles based on weights in the current particle cloud."""
        # Make sure particles are normalized
        self.normalize_particles()
        # Select new particles based on particle weight. The higher the
        # weight of a particle, the more likely it is to be chosen.
        weights = [p.w for p in self.particle_cloud]
        new_particles = draw_random_sample(
            self.particle_cloud, np.array(weights), self.n_particles
        )
        # These new particles are exact duplicates of ones that already exist in
        # the current particle cloud. We introduce normal noise for more diversity.
        for p in new_particles:
            x_noise = np.random.normal(0, 0.03)
            y_noise = np.random.normal(0, 0.03)
            theta_noise = np.random.normal(0, 0.2)
            p.x += x_noise
            p.y += y_noise
            p.theta += theta_noise % (2 * np.pi)
            p.w = 1 / self.n_particles

        self.particle_cloud = new_particles

    def update_particles_with_laser(self, r, theta):
        """Updates the particle weights in response to the scan data
        r: the distance readings to obstacles
        theta: the angle relative to the robot frame for each corresponding reading
        """

        for particle in self.particle_cloud:

            robot_theta = particle.theta
            particle.w = 0
            # Process each of the 360 laser scan points
            for i in range(len(r)):
                # If the laser scan point is invalid, ignore it.
                if np.isinf(r[i]):
                    continue
                # Calculate where the laser scan point location is in world frame
                laser_theta = theta[i]
                scan_pt_x = particle.x + r[i] * math.cos(robot_theta + laser_theta)
                scan_pt_y = particle.y + r[i] * math.sin(robot_theta + laser_theta)
                # Find distance to the closest obstacle from the scan point location.
                # If it matches the map, the distance should be 0 since the scan
                # should overlap with the obstacle.
                closest_obstacle_dist = (
                    self.occupancy_field.get_closest_obstacle_distance(
                        scan_pt_x, scan_pt_y
                    )
                )
                # Process closest obstacle distance through a gaussian function
                # centered around 0 and with y range between 0-1.
                if not np.isnan(closest_obstacle_dist):
                    particle.w += gaussian(closest_obstacle_dist, 0, 0.1) ** 3

        self.normalize_particles()
        # Publish particle weights for visualization
        self.publish_particle_weight_x_y()

    def update_initial_pose(self, msg):
        """Callback function to handle re-initializing the particle filter based on a pose estimate.
        These pose estimates could be generated by another ROS Node or could come from the rviz GUI"""
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                  particle cloud around.  If this input is omitted, the odometry will be used"""
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose
            )
        self.particle_cloud = []
        # Creates uniform gaussian distribution around point. initial_particle_spread is the
        # standard deviation and determines the spread of the initial particle cloud
        x_list = np.random.normal(
            xy_theta[0], self.initial_particle_spread, self.n_particles
        )
        y_list = np.random.normal(
            xy_theta[1], self.initial_particle_spread, self.n_particles
        )
        theta_list = np.random.uniform(0, 2 * math.pi, self.n_particles)
        weight = 1 / self.n_particles
        for i in range(self.n_particles):
            # Create particle
            particle = Particle(x_list[i], y_list[i], theta_list[i], weight)
            # Add to particle list
            self.particle_cloud.append(particle)

        self.normalize_particles()

    def normalize_particles(self):
        """Make sure the particle weights define a valid distribution (i.e. sum to 1.0)"""
        # Get sum of weights
        particle_weight_sum = sum(p.w for p in self.particle_cloud)

        if particle_weight_sum == 0:
            # Set all particles to be equal weight if the weight sum is 0
            for p in self.particle_cloud:
                p.w = 1 / self.n_particles
        else:
            # Normalize particles by multiplying it by a scale factor
            scale = 1 / particle_weight_sum  # Get scale
            for i in range(self.n_particles):
                self.particle_cloud[i].w = self.particle_cloud[i].w * scale

    def publish_particles(self, timestamp):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(
            PoseArray(
                header=Header(stamp=timestamp, frame_id=self.map_frame),
                poses=particles_conv,
            )
        )

    def publish_particle_weight_x_y(self):
        """Publish the particles' xy positions and weights so they can be visualized
        in a different node.
        """
        msg_weights = Float32MultiArray()
        msg_x = Float32MultiArray()
        msg_y = Float32MultiArray()
        msg_weights.data = [p.w for p in self.particle_cloud]
        msg_x.data = [p.x for p in self.particle_cloud]
        msg_y.data = [p.y for p in self.particle_cloud]

        self.particle_weight_publisher.publish(msg_weights)
        self.particle_x_publisher.publish(msg_x)
        self.particle_y_publisher.publish(msg_y)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
