/*
 * Map2D.cpp
 *
 *  Created on: 7 Dec 2015
 *      Author: Konrad Cop
 *
 *      The class for storing the data of the map
 */

#include "Map2D.h"

Map2D::Map2D()
{
	resolution=0;
	width=0;
	height=0;
	origin_x=0;
	origin_y=0;
}

Map2D::~Map2D() {
	// TODO Auto-generated destructor stub
}

// function for accessing the occupancy at a certain cell on the map
int Map2D::GetOccupancyAt(float x, float y)
{
	int location;
	location=int((y/resolution)*width+(x/resolution));
	return data.at(location);
}

