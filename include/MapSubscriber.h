/*
 * MapSubscriber.h
 *
 *  Created on: 3 Dec 2015
 *      Author: Konrad Cop
 *
 *      Class which subscribes the map message and save the data to the map class
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_MAPSUBSCRIBER_H_
#define COPK_ASSIGNMENT_4_INCLUDE_MAPSUBSCRIBER_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>

#include "Map2D.h"

using namespace std;

class MapSubscriber
{
private:
	Map2D* map;
public:
	MapSubscriber(Map2D*);
	virtual ~MapSubscriber();

	void callback(const nav_msgs::OccupancyGridConstPtr&);
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_MAPSUBSCRIBER_H_ */
