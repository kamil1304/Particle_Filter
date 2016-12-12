/*
 * Parameters.cpp
 *
 *  Created on: 5 Dec 2015
 *      Author: Konrad Cop
 *
 * The class which is a container for all parameters necessary for the algorithm, the parameters are loaded during the launch of the application
 */

#include "Parameters.h"

Parameters::Parameters(ros::NodeHandle * node)
{
	stage=true;
	node_handle=node;

	// default values initialization
	map_subscriber="/map";
	motion_update_subscriber="/thymio_driver/odometry";
	scan_subscriber="/scan";
	particles_publisher="/particles";
	particles_frame_id= "odometry_link";
	msg_buffer=1;

	number_of_particles=50;

	min_x=0.2;
	max_x= 0.5;
	min_y= 0.2;
	max_y= 0.5;
	min_theta= -1.5;
	max_theta= 1.5;

	alfa1= 0.02;
	alfa2= 0.02;
	alfa3= 0.02;
	alfa4= 0.02;

	outliers_threshold=15;
	scan_filter=10;

	weight_factor=100;
	rounding_factor=0.997;
	occupancy_value=100;

	localization_accuracy_radius=0.12;

	// loading parameters form file
	GetParameters();
}

Parameters::~Parameters() {
	// TODO Auto-generated destructor stub
}

// stage accessors
bool Parameters::GetStage()
{
	return stage;
}

void Parameters::SetStage(bool input)
{
	stage=input;
}

// loading parameters form file
void Parameters::GetParameters()
{
	if (!( node_handle->getParam("map_subscriber", map_subscriber))) ROS_ERROR("Failed to get param 'map_subscriber'");
	if (!( node_handle->getParam("motion_update_subscriber", motion_update_subscriber))) ROS_ERROR("Failed to get param 'motion_update_subscriber'");
	if (!( node_handle->getParam("scan_subscriber", scan_subscriber))) ROS_ERROR("Failed to get param 'scan_subscriber'");
	if (!( node_handle->getParam("particles_publisher", particles_publisher))) ROS_ERROR("Failed to get param 'particles_publisher'");
	if (!( node_handle->getParam("particles_frame_id", particles_frame_id))) ROS_ERROR("Failed to get param 'particles_frame_id'");
	if (!( node_handle->getParam("msg_buffer", msg_buffer))) ROS_ERROR("Failed to get param 'msg_buffer'");

	if (!( node_handle->getParam("number_of_particles", number_of_particles))) ROS_ERROR("Failed to get param 'number_of_particles'");
	if (!( node_handle->getParam("min_x", min_x))) ROS_ERROR("Failed to get param 'min_x'");
	if (!( node_handle->getParam("max_x", max_x))) ROS_ERROR("Failed to get param 'max_x'");
	if (!( node_handle->getParam("min_y", min_y))) ROS_ERROR("Failed to get param 'min_y'");
	if (!( node_handle->getParam("max_y", max_y))) ROS_ERROR("Failed to get param 'max_y'");
	if (!( node_handle->getParam("min_theta", min_theta))) ROS_ERROR("Failed to get param 'min_theta'");
	if (!( node_handle->getParam("max_theta", max_theta))) ROS_ERROR("Failed to get param 'max_theta'");

	if (!( node_handle->getParam("alfa1", alfa1))) ROS_ERROR("Failed to get param 'alfa1'");
	if (!( node_handle->getParam("alfa2", alfa2))) ROS_ERROR("Failed to get param 'alfa2'");
	if (!( node_handle->getParam("alfa3", alfa3))) ROS_ERROR("Failed to get param 'alfa3'");
	if (!( node_handle->getParam("alfa4", alfa4))) ROS_ERROR("Failed to get param 'alfa4'");

	if (!( node_handle->getParam("outliers_threshold", outliers_threshold))) ROS_ERROR("Failed to get param 'outliers_threshold'");
	if (!( node_handle->getParam("scan_filter", scan_filter))) ROS_ERROR("Failed to get param 'scan_filter'");

	if (!( node_handle->getParam("weight_factor", weight_factor))) ROS_ERROR("Failed to get param 'weight_factor'");
	if (!( node_handle->getParam("rounding_factor", rounding_factor))) ROS_ERROR("Failed to get param 'rounding_factor'");
	if (!( node_handle->getParam("occupancy_value", occupancy_value))) ROS_ERROR("Failed to get param 'occupancy_value'");

	if (!( node_handle->getParam("localization_accuracy_radius", localization_accuracy_radius))) ROS_ERROR("Failed to get param 'localization_accuracy_radius'");
}
