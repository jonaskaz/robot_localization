# Robot Localization

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


### Update the weight of the particles based on laser scan
To weight the particles, we went through each particle in the cloud and performed the following steps:
- Go through each laser scan point from the robot and for each scan point:
- Calculate the laser scan point location as if the robot had taken it from the particle's position
- Find the distance to the closest obstacle to that laser scan point
- Pass that distance through a gaussian function, and add it to the weight of the particle
With this setup, particles that have laser scans with very small closest obstacles will be weighted highly, wheras particles with laser scans that are far off from the map will have lower weights. We can adjust the standard deviation of our gaussian function in order to either increase or decrease the distance between lower and highly weighted particles

## Resample the particles
To resample the particles, we simply draw a random sample from all the particles using the particle's weight to 

How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).

## Design Decision
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Challenges
What if any challenges did you face along the way?

## Future Improvements
What would you do to improve your project if you had more time?

## Reflection
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.



