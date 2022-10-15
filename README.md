# Robot Localization

## Introduction
Localization is one of the core challenges to robotics. While many solutions
rely on external sensors, such as GPS, these tend to be expensive and require
manual setup in the environment the robot operates in. The localization
algorithm we will be exploring in this project is a particle filter, a
completely self-contained system using only a lidar sensor.

## Purpose
The goal of our project was the explore robot localization by implementing the particle filter algorithm. This work helped us build more fluency with ROS and robotics debugging.

## Implementation
The main steps of our algorithm were:
- Initialize a particle cloud
- Update the position of the particles based on the odometry data
- Update the weight of the particles based on the robot's laser scan
- Resample the particles

### Initialize the particle cloud
To intialize the particles, we drew points from a uniform gaussian distribution centered around the intial pose. With this method, we can adjust the standard deviation of the distribution in order to control the spread of the particles when initialized.

### Update particle position based on odometry
Every time we got an odometry update, each particle's pose was updated to follow
the same transformation the robot did. To do so, we started by representing each
particle pose as a transformation of a unit vector at the origin of the world
frame. Then, we computed the transformation matrix between the two robot poses
in the odometry frames. This transformation matrix was then applied to the
particle pose, effectively moving the particle pose by the same amount the robot
moved.

The transformation matrix for a general pose is:

![transformation matrix](img/transformation_matrix.svg)

This transforms a unit vector along the x axis to the specified pose. This is
especially useful because the same form can then represent a pose and also be
used to represent the transformation between poses.

To compute the transformation to apply, we start by representing the two robot
poses (original and updated) in the transformation matrix format. We can then
use the dot product of these two products to compute the transformation. We
use the inverse of the original robot position because this effectively
transforms a pose from the original robot position to the odometry origin, then
from the odometry origin to the final robot position.

![particle pose transformation](img/particle_pose_transformation.svg)

The transformation matrices from the origin to the robot pose can be computed by
using the robot pose in the odometry frame. Because we are transforming to the
origin and then from the origin, the coordinate system origin we use does not
matter, and the odometry frame is the most convienent here.

### Update the weight of the particles based on laser scan
To weight the particles, we went through each particle in the cloud and performed the following steps:
- Go through each laser scan point from the robot and for each scan point:
- Calculate the laser scan point location as if the robot had taken it from the particle's position
- Find the distance to the closest obstacle to that laser scan point
- Pass that distance through a gaussian function, and add it to the weight of the particle
With this setup, particles that have laser scans with very small closest obstacles will be weighted highly, wheras particles with laser scans that are far off from the map will have lower weights. We can adjust the standard deviation of our gaussian function in order to either increase or decrease the distance between lower and highly weighted particles

## Resample the particles
To resample the particles, we simply draw a random sample from all the particles using the particle's weight as a bias. This makes the algorithm choose the heavily weighted (ie more likely) particles more often, which causes the particles to converge. However, because the highly weighted particles can (and probably will) be sampled multiple times, we also add in noise, which is sampled from a normal distribution. This noise ensures that the new particles have a bit of diversity, which keeps the particle filter from converging in the wrong spot too quickly.

## Demo
Below you can see our our particle algorithm localize the position of a neato robot in a small map. The particles are represented by small red arrows, the robot is represented by a large red arrow, and the laser scan is represented by red dots. We can see that as the robot drives around, particles begin to form a cohesive group, and the laser scan begins to align with map.  
![small map robot localization](/img/robot_localization_gauntlet.gif)

We used the same visualization approach to localize the robot in a larger map. However, for this map we increased our intial particle spread. This spread is shown in the image below.  
<img src="img/init_particles.jpg" alt="initial particle spread" width="300"/>

Below we can see our algorithm localizing the robot in a larger map.  
![full map robot localization](/img/robot_localization_mac.gif)

## Design Decision

To determine the robot's position from the particle cloud, we had the option of either taking the weighted mean pose of all the particles or the pose of the best (most heavily weightd) particle. After experimenting with both approaches, we decided to use the mean pose because it led to a smoother motion of the robot.

Another decision decision we made was in the function to calculate the weight for each particle. After obtaining the distance from each of the 360 laser scans points to the closest obstacle, we had several options for weight calculation: 
* Set a certain threshold (Ex: 0.05 meters) to consider a laser scan point "accurate", and weigh each particle based on how many scan points are accurate out of 360.
* Take the sum of the distances and obtain the inverse, such that if the summed distance to the closest obstacles is small, it is weighted more. 
* Put each distance to the closest obstacle into a gaussian function centered at 0 with y values from 0-1 and multiply the outputs of the guassian function together.

We decided to use the gaussian function method because it allowed us to weigh more accurate particle more dramatically and adjust the rate of convergence by modifying the standard deviation. 

## Challenges

Some challenges we faced for this project include accurately visualizing each step and understanding the mathematical aspects like transformations. 

We first tried to implement the entire particle filter without testing, but after failing to localize the robot, we realized we needed to test each step. This required us to backtrack and downscale to 3 particles and make sure that the particles updated with the robot movement, adjusted weights, and resampled all as expected. It was tricky figuring out how to best visualize the weights but we eventually went with a scatter plot with size of the plot points varying based on weight. 

Understanding the transforms used in the update particles with odometry section was also difficult. As a team, we walked through the math together on a whiteboard to wrap our heads around the different frames. 

## Future Improvements

If we had more time in this project, we would like to experiment with speeding up the particle filter by replacing a lot of the for loops with matrix multiplication. This will allow us to tackle bigger localizations challenges without the lag that we currently experience. 

## Reflection

From this project, one of our key takeaways is to test frequently and often. As a team, we ended up spending much more time than needed debugging because we didn't test each step properly in the beginning. 



