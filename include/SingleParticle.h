/*
 * SingleParticle.h
 *
 *  Created on: 4 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class which is an abstraction of a single particle - it has its pose, and functionality to update the motion
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_SINGLEPARTICLE_H_
#define COPK_ASSIGNMENT_4_INCLUDE_SINGLEPARTICLE_H_

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include "Parameters.h"

using namespace std;

class SingleParticle {
private:
	geometry_msgs::Pose2D pose, previous_pose;//poses of the particle
	Parameters* params;
public:
	SingleParticle();
	SingleParticle(float, float, float, Parameters*);
	virtual ~SingleParticle();

	void UpdatePose(geometry_msgs::Pose2D, geometry_msgs::Pose2D);
	void SavePreviousPose();
	void AssignPose(geometry_msgs::Pose2D);

	geometry_msgs::Pose2D GetPose();
	geometry_msgs::Pose2D GetPreviousPose();
	float NormalSample(float, float, float, float, float, float);
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_SINGLEPARTICLE_H_ */
