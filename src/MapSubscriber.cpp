/*
 * MapSubscriber.cpp
 *
 *  Created on: 3 Dec 2015
 *      Author: Konrad Cop
 *
 *   Class which subscribes the map message and save the data to the map class
 */

#include "MapSubscriber.h"

MapSubscriber::MapSubscriber(Map2D* m)
{
	map=m;
}

MapSubscriber::~MapSubscriber() {
	// TODO Auto-generated destructor stub
}

// getting information about the map from ros message and saving it to the object Map2D
void MapSubscriber::callback(const nav_msgs::OccupancyGridConstPtr&  map_msg)
{
	map->width=map_msg->info.width;
	map->height=map_msg->info.height;
	map->resolution=map_msg->info.resolution;
	map->origin_x=map_msg->info.origin.position.x;
	map->origin_y=map_msg->info.origin.position.y;

	// storage of data
	for(int i=0; i<map_msg->data.size(); i++)
	{
		map->data.push_back(map_msg->data.at(i));
	}

	ROS_INFO("map size: [%d]", map_msg->data.size());
}

