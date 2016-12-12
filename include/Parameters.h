/*
 * Parameters.h
 *
 *  Created on: 5 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class which is a container for all parameters necessary for the algorithm, the parameters are loaded during the launch of the application
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_PARAMETERS_H_
#define COPK_ASSIGNMENT_4_INCLUDE_PARAMETERS_H_

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <string>

using namespace std;

class Parameters {
private:
	bool stage; // the logical condition - enables sensor update to wait for the motion update and vice versa
	ros::NodeHandle * node_handle;
public:
	Parameters(ros::NodeHandle *);
	virtual ~Parameters();
	void GetParameters();
	bool GetStage();
	void SetStage(bool);

	string map_subscriber, motion_update_subscriber, scan_subscriber,particles_publisher, particles_frame_id;
	int number_of_particles, outliers_threshold, scan_filter;
	int occupancy_value, msg_buffer;
	double min_x, max_x, min_y, max_y, min_theta, max_theta; // range of positions and orientations of initial particles
	double alfa1, alfa2, alfa3, alfa4; // noise parameters for motion
	double weight_factor, rounding_factor;
	double localization_accuracy_radius;
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_PARAMETERS_H_ */
