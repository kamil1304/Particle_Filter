/*
 * MotionUpdate.cpp
 *
 *  Created on: 3 Dec 2015
 *      Author: Konrad Cop
 *
 *      The main class for motion update - it applies the odometry message to every single particle and disturb it with noise
 */

#include "MotionUpdate.h"

MotionUpdate::MotionUpdate( Particles* part, Parameters* p, ParticlesPublisher* part_p)
{
	params=p;
	particles_ptr=part;
	part_publisher=part_p;
}

MotionUpdate::~MotionUpdate() {
	// TODO Auto-generated destructor stub
}

// repeating the procedure in the loop
void MotionUpdate::callback(const nav_msgs::OdometryConstPtr& odo_msg)
{
	while(params->GetStage() && ros::ok())
	{
		particles_ptr->SavePreviousPoses(); // assigning pose from the previous iteration
		particles_ptr->UpdatePose(odo_msg); // updating pose by incorporating the odometry change
		part_publisher->PublishParticles();// publishing particles in rviz
		if(particles_ptr->IsAccurate())
		{
			part_publisher->PublishLocalized();
		}

		// switch the stage - sensor update is activated, motion update waits
		params->SetStage(false);
	}
}

