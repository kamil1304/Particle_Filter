/*
 * localization.cpp
 *
 *  Created on: 3 Dec 2015
 *      Author: Konrad Cop
 *
 *      The main function for a particle filter node - in it all the object, publishers and subscribers are initialized
 */

#include <ros/ros.h>
#include <iostream>
#include <ros/callback_queue.h>

#include "geometry_msgs/PoseArray.h"
#include <vector>

#include "MapSubscriber.h"
#include "MotionUpdate.h"
#include "Particles.h"
#include "SensorUpdate.h"
#include "SingleParticle.h"
#include "Parameters.h"
#include "ParticlesPublisher.h"
#include "Map2D.h"

using namespace std;

int main(int argc, char** argv)
{
	//node initialization
	ros::init(argc, argv, "localization");

	// node handles
	ros::NodeHandle map_handle, motion_handle, nh, sensor_handle;
	ros::NodeHandle* ptr_handle=&map_handle;

	// particles publisher
	ros::Publisher pub, pub_one;
	ros::Publisher* pub_ptr=&pub;
	ros::Publisher* pub_one_ptr=&pub_one;

	// parameters of the algorithm
	Parameters params(ptr_handle);
	Parameters* params_ptr=&params;

	//map
	Map2D map;
	Map2D* map_ptr=&map;

	// getting the map
	MapSubscriber map_subscriber(map_ptr);
	MapSubscriber* mapsub_ptr=&map_subscriber;

	// creating set of particles
	Particles particles(params_ptr);
	Particles* particles_ptr=&particles;

	// particles publisher
	ParticlesPublisher particles_publisher(particles_ptr, pub_ptr, pub_one_ptr, params_ptr);
	ParticlesPublisher* part_pub_ptr=&particles_publisher;

	// steps of algorithm
	MotionUpdate motion(particles_ptr, params_ptr, part_pub_ptr);
	SensorUpdate sensor(map_ptr, params_ptr, particles_ptr);

	// ros subscribers
	ros::Subscriber sub1 = map_handle.subscribe(params.map_subscriber, params.msg_buffer,  &MapSubscriber::callback, &map_subscriber); // map
	ros::Subscriber sub2= motion_handle.subscribe(params.motion_update_subscriber, params.msg_buffer,  &MotionUpdate::callback, &motion); // motion update (odometry subscription)
	ros::Subscriber sub3= sensor_handle.subscribe(params.scan_subscriber, params.msg_buffer,  &SensorUpdate::callback, &sensor); // sensor update (scan subscription)

	// ros publisher - particles
	pub = map_handle.advertise<geometry_msgs::PoseArray> (params.particles_publisher, params.msg_buffer);
	pub_one= map_handle.advertise<geometry_msgs::PoseStamped> ("one_position", 1000);

	ros::spin();
	return 0;
}

