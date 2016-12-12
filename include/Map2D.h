/*
 * Map2D.h
 *
 *  Created on: 7 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class for storing the data of the map
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_MAP2D_H_
#define COPK_ASSIGNMENT_4_INCLUDE_MAP2D_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>

using namespace std;

class Map2D {

public:
	Map2D();
	virtual ~Map2D();

	vector<int> data; // occupancy data vector
	float resolution;
	float width, height;
	float origin_x, origin_y;
	int GetOccupancyAt(float, float);
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_MAP2D_H_ */
