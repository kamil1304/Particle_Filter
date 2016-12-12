/*
 * SingleParticle.cpp
 *
 *  Created on: 4 Dec 2015
 *      Author: Konrad Cop
 *
 * The class which is an abstraction of a single particle - it has its pose, and functionaltiy to update the motion
 */

#include "SingleParticle.h"
SingleParticle::SingleParticle()
{
	pose.x=0;
	pose.y=0;
	pose.theta=0;
	params=NULL;
}
SingleParticle::SingleParticle(float x, float y, float theta, Parameters* p)
{
	pose.x=x;
	pose.y=y;
	pose.theta=theta;
	params=p;
}

SingleParticle::~SingleParticle() {
	// TODO Auto-generated destructor stub
}

void SingleParticle::UpdatePose(geometry_msgs::Pose2D current_pose_msg, geometry_msgs::Pose2D previous_pose_msg)
{
	float drot1, drot2, dtrans;
	float drot1_err, drot2_err, dtrans_err;

	// calculating the motion increments
	drot1=atan2(current_pose_msg.y-previous_pose_msg.y,current_pose_msg.x-previous_pose_msg.x)-previous_pose_msg.theta;
	dtrans=sqrt((current_pose_msg.x-previous_pose_msg.x)*(current_pose_msg.x-previous_pose_msg.x)+(current_pose_msg.y-previous_pose_msg.y)*(current_pose_msg.y-previous_pose_msg.y));
	drot2=current_pose_msg.theta-previous_pose_msg.theta-drot1;

	// disturbing the motion with the noise
	drot1_err=drot1-NormalSample(params->alfa1,drot1,params->alfa2, dtrans, 0, params->alfa1);
	dtrans_err=dtrans-NormalSample(params->alfa3, dtrans, params->alfa4, drot1, params->alfa4, drot2);
	drot2_err=drot2-NormalSample(params->alfa1, drot2, params->alfa2, dtrans,0, params->alfa1);

	// assigning a new position to the particle
	pose.x=previous_pose.x+dtrans_err*cos(previous_pose.theta+drot1_err);
	pose.y=previous_pose.y+dtrans_err*sin(previous_pose.theta+drot1_err);
	pose.theta=previous_pose.theta+drot1_err+drot2_err;
}

// calculating the noise of the particle, sampled form the normal distribution
float SingleParticle::NormalSample(float a1, float p1, float a2, float p2, float a3, float p3)
{
	float variable, random_num;
	variable=a1*p1*p1+a2*p2*p2+a3*p3*p3;
	variable=sqrt(variable);
	float sum=0;
	for(int i=0; i<12; i++)
	{
		float temp;
		temp=(float(rand()) / float(RAND_MAX)) *2 -1;
		sum=sum+temp;
	}
	return (variable*sum)/6;
}

void SingleParticle::AssignPose(geometry_msgs::Pose2D input_pose)
{
	pose=input_pose;
}

void SingleParticle::SavePreviousPose()
{
	previous_pose=pose;
}

// accessors
geometry_msgs::Pose2D SingleParticle::GetPose()
{
	return pose;
}

geometry_msgs::Pose2D SingleParticle::GetPreviousPose()
{
	return previous_pose;
}
