/*
 * MotionUpdate.h
 *
 *  Created on: 3 Dec 2015
 *      Author: Konrad Cop
 *
 *	The main class for motion update - it applies the odometry message to every single particle and disturb it with noise
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_MOTIONUPDATE_H_
#define COPK_ASSIGNMENT_4_INCLUDE_MOTIONUPDATE_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

#include "Particles.h"
#include "Parameters.h"
#include "ParticlesPublisher.h"

using namespace std;

class MotionUpdate {
private:
	Particles* particles_ptr;
	Parameters* params;
	ParticlesPublisher* part_publisher;
public:
	MotionUpdate(Particles*, Parameters*, ParticlesPublisher*);
	virtual ~MotionUpdate();

	void callback(const nav_msgs::OdometryConstPtr&);
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_MOTIONUPDATE_H_ */
