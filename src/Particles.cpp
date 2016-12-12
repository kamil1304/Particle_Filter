/*
 * Particles.cpp
 *
 *  Created on: 4 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class which creates the whole set of particles (upper class for a single particle)
 */

#include "Particles.h"

Particles::Particles(Parameters* p)
{
	params=p;

	// parameters initialization
	particles_amount=params->number_of_particles;
	min_x=params->min_x;
	max_x=params->max_x;
	min_y=params->min_y;
	max_y=params->max_y;
	min_theta=params->min_theta;
	max_theta=params->max_theta;

	weights.resize(particles_amount);

	// random initialization of the particles on the whole map
	Initialize();
	InitialWeights();
}

Particles::~Particles() {
	// TODO Auto-generated destructor stub
}

// the function initializes the particles with uniform distribution within the declared range
void Particles::Initialize()
{
	for(int i=0; i<particles_amount; i++)
	{
		float x=(float(rand()) / float(RAND_MAX)) * (max_x - min_x) + min_x;
		float y=(float(rand()) / float(RAND_MAX)) * (max_y - min_y) + min_y;
		float theta=(float(rand()) / float(RAND_MAX)) * (max_theta - min_theta) + min_theta;
		SingleParticle temp_particle(x,y,theta, params);
		particles.push_back(temp_particle);
	}
	ROS_INFO("[%d] random particles initialized", particles.size());
}

// updating the pose of each particle - the update function in a single particle is called
void Particles::UpdatePose(const nav_msgs::OdometryConstPtr& current_msg)
{
	current_pose=current_msg->pose;
	current_pose2d=ComputePose(current_msg->pose);
	for(int i=0; i<particles.size();i++)
	{
		particles.at(i).UpdatePose(current_pose2d, previous_pose2d);
	}
}

// pose transformation
geometry_msgs::Pose2D Particles::ComputePose(geometry_msgs::PoseWithCovariance odo_msg)
{
	geometry_msgs::Pose2D temp_pose;
	float quaternion_0,quaternion_1, quaternion_2, quaternion_3;

	quaternion_0=odo_msg.pose.orientation.w;
	quaternion_1=odo_msg.pose.orientation.x;
	quaternion_2=odo_msg.pose.orientation.y;
	quaternion_3=odo_msg.pose.orientation.z;

	temp_pose.x=odo_msg.pose.position.x;
	temp_pose.y=odo_msg.pose.position.y;
	temp_pose.theta=atan2(2*(quaternion_0*quaternion_3+quaternion_1*quaternion_2), 1-2*(quaternion_2*quaternion_2+quaternion_3*quaternion_3));

	return temp_pose;
}

// saving the previous pose - to be called before updating to the new one
void Particles::SavePreviousPoses()
{
	previous_pose=current_pose;
	previous_pose2d=current_pose2d;
	for(int i=0; i<particles.size(); i++)
	{
		particles.at(i).SavePreviousPose();
	}
}

// function used in resampling - replaces initial particle with the desired one
void Particles::ReplaceParticle(int initial, int desired)
{
	particles.at(initial)=particles.at(desired);
}

// assigning initial weights for every particle
void Particles::InitialWeights()
{
	min_weight=1/float(params->weight_factor*weights.size());
	for(int i=0; i<weights.size();i++)
	{
		weights.at(i)=min_weight;
	}
}

// access for changing the weight from outside function
void Particles::AssignWeight(float weight, int particle_index)
{
	weights.at(particle_index)=weight;
}

// ensuring that the weights sums up to 1 (probability can't be bigger)
void Particles::NormalizeWeights()
{
	float normalization=0;// normalization factor
	sum_of_weights=0;

	for(int i=0; i<weights.size();i++)
	{
		normalization=float(normalization)+float(weights.at(i));
	}
	for (int i=0; i<weights.size(); i++)
	{
		weights.at(i)=weights.at(i)/normalization;
		sum_of_weights=sum_of_weights+weights.at(i);
	}
}

// function for changing the whole set of particles into one position
void Particles::DetermineLocalizedPose()
{
	int highest_weight_index=0;
	float temp_weight = 0;
	for(int i=0; i<weights.size();i++)
	{
		if (weights.at(i)>temp_weight)
		{
			highest_weight_index=i;
			temp_weight=weights.at(i);
		}
	}
	localized_pose.AssignPose(particles.at(highest_weight_index).GetPose());
}

bool Particles::IsAccurate()
{
	double r=params->localization_accuracy_radius;
	double a,b, circle;
	bool is=false;
	for(int i=0; i<weights.size();i++)
	{
		a=localized_pose.GetPose().x-particles.at(i).GetPose().x;
		b=localized_pose.GetPose().y-particles.at(i).GetPose().y;
		circle=a*a + b*b;


		if ( circle> r*r)
		{
			is=false;
			break;
		}
		else
		{
			is = true;
		}
	}
	return is;
}

// accessors
float Particles::GetWeight(int particle_index)
{
	return weights.at(particle_index);
}

float Particles::GetMinimumWeight()
{
	return min_weight;
}

geometry_msgs::PoseWithCovariance Particles::GetCurrentPose()
{
	return current_pose;
}

geometry_msgs::PoseWithCovariance Particles::GetPreviousPose()
{
	return previous_pose;
}


