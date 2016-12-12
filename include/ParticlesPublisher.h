/*
 * ParticlesPublisher.h
 *
 *  Created on: 7 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class for publishing the particles in rviz
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_PARTICLESPUBLISHER_H_
#define COPK_ASSIGNMENT_4_INCLUDE_PARTICLESPUBLISHER_H_

#include <ros/ros.h>
#include "Particles.h"
#include "Parameters.h"
#include "geometry_msgs/PoseStamped.h"

class ParticlesPublisher {
private:
	Particles* particles;
	ros::Publisher* publisher;
	ros::Publisher* publisher_localized;
	Parameters* params;
	geometry_msgs::PoseArray particles_message; // array to be published
	geometry_msgs::PoseStamped localized_particle;
public:
	ParticlesPublisher(Particles*, ros::Publisher*,ros::Publisher*, Parameters*);
	virtual ~ParticlesPublisher();

	void PublishParticles(); // publishing in rviz
	void PublishLocalized();
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_PARTICLESPUBLISHER_H_ */
