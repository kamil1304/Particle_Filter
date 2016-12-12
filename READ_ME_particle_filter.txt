-------------- Functionality ---------------------------------------------
The application is the implementation of a particle filter. Having declared the number of particles, the area of the map, noise coefficients and other parameters, the program locates the robot within the map. For that the control input and the measurement values are used. As a result the pose of the robot is determined, as a set of particles located around the most probable point. 

------------ Usage and working principles ----------------------------------
Before running, the algoritm parameters have to be set in the parameters yaml file. The application is run as a ROS node, and for that the provided launch file is used.
In the main function of the application the map object is created by subscription from the map topic. The set of particles is created as a single object, which consists of the vector of single particles. At the beginning, the particles are initialized randomly within the declared range, and each of them has assigned the same weight - min_weight. 
The procedure consists of 2 steps:
	#In motion update - each particle is moved according to the odometry disturbed with a noise.(previous pose is stored for comparison purposes).

	#In sensor update, the location obtained from odometry is verified by the easurment from the laser scanner. The points from the scanner are compared with the provided map, and for each particle the correlation is computed. Additionally, the read points that lay outside the map, and particles also outside, are not considered for further filtering (the minimal weight is assigned for them). The weight of each particles is a number of correspondences normalized, so that the probability sums up to 1. 
After comparison the articles are resampled using roulette resampling, so that the particles with the biggest probability are considered in the further steps. 

The calculated, scaled and resampled particles are then visualized in rviz, having the certain position and orientation.

------------ Directory structure ---------------------------------------
localization.cpp - main class of the node in which main objects are created
Parameters.cpp - class for storing the algorithm parameters
Map2D.cpp - map of the environment
MapSubscriber.cpp - subscribing a map from the map topic
Particles.cpp - set of particles with several functionalities
SingleParticle.cpp - an abstraction of a single particle
ParticlesPublisher.cpp - class for displaying the particles in rviz
MotionUpdate.cpp - first step of algorithm realised in the callback
SensorUpdate.cpp - class for using the scans, weight calculation and resampling realised in the callback

