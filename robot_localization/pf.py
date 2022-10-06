#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

from operator import attrgetter
import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed

class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
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
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(PoseArray, "particlecloud", qos_profile_sensor_data)

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
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        # print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # updates weight based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()
        # Jackie's code below
        # Compute the mean pose by incorporating the weights
        x_mean = sum(p.x*p.w for p in self.particle_cloud)
        y_mean = sum(p.y*p.w for p in self.particle_cloud)
        theta_mean = sum(p.theta*p.w for p in self.particle_cloud)     
        mean_pose = Particle(x_mean, y_mean, theta_mean)
        self.robot_pose = mean_pose.as_pose()
        self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                self.odom_pose)

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0], # x delta
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1], # y delta
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) # theta delta

            # Create transformation matrix from position 1 to position 2
            c,s = np.cos(self.current_odom_xy_theta[2]), np.sin(self.current_odom_xy_theta[2])
            T_current2odom = np.array( ((c,-s, self.current_odom_xy_theta[0]), 
                                        (s, c, self.current_odom_xy_theta[1]), 
                                        (0, 0, 1)))
            c,s = np.cos(new_odom_xy_theta[2]), np.sin(new_odom_xy_theta[2])
            T_new2odom = np.array(     ((c,-s, new_odom_xy_theta[0]), 
                                        (s, c, new_odom_xy_theta[1]), 
                                        (0, 0, 1)))             
            T_new2current = np.linalg.inv(T_current2odom) @ T_new2odom

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # for particle in self.particle_cloud:
        for particle in self.particle_cloud[0:360]:
            c,s = np.cos(particle.theta), np.sin(particle.theta)
            # create transformation matrix for particle
            T_particle = np.array((particle.x, particle.y, 1))
            new_particle_loc = np.dot(T_new2current, T_particle)
            # print("new_particle_loc:", new_particle_loc)
            # print("OLD: x, y, theta:" ,particle.x, particle.y, particle.theta)
            
            particle.x = new_particle_loc[0]
            particle.y = new_particle_loc[1]
            particle.theta = particle.theta + delta[2]
            # print("UPDATE: x, y, theta:" ,particle.x, particle.y, particle.theta)
            # particle.x = particle.x + delta[0]
            # particle.y = particle.y + delta[1]
            # particle.theta = particle.theta + delta[2]
            # TODO: add in noise? zero mean gaussian noise, check the textbook to see the best way

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # TODO: fill out the rest of the implementation
        # Resample based on weight of particle
        x_choices = [p.x for p in self.particle_cloud]
        y_choices = [p.y for p in self.particle_cloud]
        theta_choices = [p.theta for p in self.particle_cloud]
        probabilities = [p.w for p in self.particle_cloud]
        x_list = draw_random_sample(x_choices, probabilities, self.n_particles)
        y_list = draw_random_sample(y_choices, probabilities, self.n_particles)
        theta_list = draw_random_sample(theta_choices, probabilities, self.n_particles)
        # Clear particle cloud
        self.particle_cloud = []
        # Add new particles to particle cloud
        for i in range(self.n_particles):
            # Create particle
            particle = Particle(x_list[i], y_list[i], theta_list[i])
            # Add to particle list
            self.particle_cloud.append(particle)

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """

        # TODO: implement this
        for particle in self.particle_cloud:
            # angle the robot is pointing relative to the world frame
            robot_theta = particle.theta
            # List for closest obstacle distances
            closest_obstacle_dist = []
            # Number of scan points that match with an obstacle in the map
            num_matching = 0
            # Calculate the closest obstacle for each data point from the laser scan
            for i in range(len(r)):
                laser_theta = theta[i]
                # Calculate laser scan point location in world frame
                scan_pt_x = particle.x - r[i]*math.sin(robot_theta+laser_theta)
                scan_pt_y = particle.y + r[i]*math.cos(robot_theta+laser_theta)
                # Find distance to the closest obstacle from the scan point
                closest_obstacle_dist = self.occupancy_field.get_closest_obstacle_distance(scan_pt_x, scan_pt_y)
                # TODO: Ask in class if there is a better way to do this
                # If distance is less than 0.1 m, we count it as accurate
                if closest_obstacle_dist <= 0.1:
                    num_matching += 1
            scan_accuracy = num_matching/360
            # Update particle weight using gaussian function centered at x=1 and y range from 0-1
            particle.w = self.gaussian(scan_accuracy, 1, 0.5)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []
        # Creates uniform gaussian distribution around p    oint.
        # random.normal(mean, standard deviation, size)
        # std = determines the spread of the initial particle cloud
        std = 0.3
        x_list = np.random.normal(xy_theta[0], std, self.n_particles)
        y_list = np.random.normal(xy_theta[1], std, self.n_particles)
        theta_list = np.random.uniform(0,2*math.pi, self.n_particles) # TODO: might be sus, check with Loren
        for i in range(self.n_particles):
            # Create particle
            particle = Particle(x_list[i], y_list[i], theta_list[i])
            # Add to particle list
            self.particle_cloud.append(particle)

        self.normalize_particles()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # Get sum of weights
        particle_weight_sum = sum(p.w for p in self.particle_cloud)
        # Divide the sum by 1 to get scale factor
        scale = 1/particle_weight_sum
        # Normalize particles with new scale
        for i in range(self.n_particles):
            self.particle_cloud[i].w = self.particle_cloud[i].w * scale

    def publish_particles(self, timestamp):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=timestamp,
                                            frame_id=self.map_frame),
                                  poses=particles_conv))

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

    def gaussian(self, x, mu, sig):
        """ Gaussian distribution function with y values between 0-1
            Arguments:
            sig: standard deviation
            mu: mean
        """
        return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()