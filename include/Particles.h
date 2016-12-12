/*
 * Particles.h
 *
 *  Created on: 4 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class which creates the whole set of particles (upper class for a single particle)
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_PARTICLES_H_
#define COPK_ASSIGNMENT_4_INCLUDE_PARTICLES_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include  <cstdlib>
#include <time.h>
#include <cmath>

#include "SingleParticle.h"
#include "Parameters.h"

using namespace std;

class Particles {
private:
	float min_x, max_x, min_y, max_y, min_theta, max_theta;// ranges of the initial area
	float min_weight;
	int particles_amount;
	Parameters* params;
	vector<float> weights;
	geometry_msgs::PoseWithCovariance current_pose, previous_pose;
	geometry_msgs::Pose2D current_pose2d, previous_pose2d;
public:
	SingleParticle localized_pose;
	Particles(Parameters*);
	virtual ~Particles();
	float sum_of_weights;
	vector<SingleParticle> particles;

	void Initialize();
	void UpdatePose(const nav_msgs::OdometryConstPtr&);
	void SavePreviousPoses();
	void ReplaceParticle(int, int);
	void AssignWeight(float,int);
	void InitialWeights();
	void NormalizeWeights();
	void DetermineLocalizedPose();

	bool IsAccurate();
	float GetWeight(int);
	float GetMinimumWeight();
	geometry_msgs::Pose2D ComputePose(geometry_msgs::PoseWithCovariance);
	geometry_msgs::PoseWithCovariance GetCurrentPose();
	geometry_msgs::PoseWithCovariance GetPreviousPose();
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_PARTICLES_H_ */
