/*
 * ParticlesPublisher.cpp
 *
 *  Created on: 7 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class for publishing the particles in rviz
 */

#include "ParticlesPublisher.h"

ParticlesPublisher::ParticlesPublisher(Particles* p, ros::Publisher* pub, ros::Publisher* pub2, Parameters* par)
{
	particles=p;
	publisher=pub;
	publisher_localized=pub2;
	params=par;
}

ParticlesPublisher::~ParticlesPublisher() {
	// TODO Auto-generated destructor stub
}

// publishing particles in rviz
void ParticlesPublisher::PublishParticles()
{
	float q0,q1,q2,q3, psi, fi, theta;
	particles_message.poses.clear();
	particles_message.header.stamp = ros::Time::now();
	particles_message.header.frame_id = params->particles_frame_id;


	geometry_msgs::Pose one_particle_msg;
	geometry_msgs::Pose2D one_particle;

	for(int i=0;i<particles->particles.size();i++)
	{
		one_particle=particles->particles.at(i).GetPose();

		// calculating the orientation of the particle
		fi=0;
		theta=0;
		psi=one_particle.theta;
		q0= cos(fi/2)*cos(theta/2)*cos(psi/2)+sin(fi/2)*sin(theta/2)*sin(psi/2);
		q1= sin(fi/2)*cos(theta/2)*cos(psi/2)-cos(fi/2)*sin(theta/2)*sin(psi/2);
		q2= cos(fi/2)*sin(theta/2)*cos(psi/2)+sin(fi/2)*cos(theta/2)*sin(psi/2);
		q3= cos(fi/2)*cos(theta/2)*sin(psi/2)-sin(fi/2)*sin(theta/2)*cos(psi/2);

		one_particle_msg.position.x=one_particle.x;
		one_particle_msg.position.y=one_particle.y;
		one_particle_msg.position.z=0;
		one_particle_msg.orientation.w=q0;
		one_particle_msg.orientation.x=q1;
		one_particle_msg.orientation.y=q2;
		one_particle_msg.orientation.z=q3;

		particles_message.poses.push_back(one_particle_msg);
	}
	publisher->publish(particles_message);
}

// publishing the most probable pose
void ParticlesPublisher::PublishLocalized()
{
	float q0,q1,q2,q3, psi, fi, theta;
	localized_particle.header.stamp = ros::Time::now();
	localized_particle.header.frame_id = params->particles_frame_id;

	fi=0;
	theta=0;
	psi=particles->localized_pose.GetPose().theta;
	q0= cos(fi/2)*cos(theta/2)*cos(psi/2)+sin(fi/2)*sin(theta/2)*sin(psi/2);
	q1= sin(fi/2)*cos(theta/2)*cos(psi/2)-cos(fi/2)*sin(theta/2)*sin(psi/2);
	q2= cos(fi/2)*sin(theta/2)*cos(psi/2)+sin(fi/2)*cos(theta/2)*sin(psi/2);
	q3= cos(fi/2)*cos(theta/2)*sin(psi/2)-sin(fi/2)*sin(theta/2)*cos(psi/2);

	localized_particle.pose.position.x=particles->localized_pose.GetPose().x;
	localized_particle.pose.position.y=particles->localized_pose.GetPose().y;
	localized_particle.pose.position.z=0;
	localized_particle.pose.orientation.w=q0;
	localized_particle.pose.orientation.x=q1;
	localized_particle.pose.orientation.y=q2;
	localized_particle.pose.orientation.z=q3;

	ROS_INFO("\nLocalized particle coordinates: [%f , %f] \n",localized_particle.pose.position.x,localized_particle.pose.position.y);

	publisher_localized->publish(localized_particle);
}
