/*
 * SensorUpdate.h
 *
 *  Created on: 5 Dec 2015
 *      Author: Konrad Cop
 *
 *  The class which perform the whole sensor update step, it assigns the weight to every particle and do the resampling afterwards
 */

#ifndef COPK_ASSIGNMENT_4_INCLUDE_SENSORUPDATE_H_
#define COPK_ASSIGNMENT_4_INCLUDE_SENSORUPDATE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <vector>
#include <math.h>

#include "Parameters.h"
#include "Particles.h"
#include "Map2D.h"

using namespace std;

class SensorUpdate {
private:
	Parameters* params;
	Particles* particles;
	Map2D* map;
public:
	SensorUpdate(Map2D*,Parameters*, Particles*);
	virtual ~SensorUpdate();

	void callback(const sensor_msgs::LaserScanConstPtr&);
	void ComputeWeight(const sensor_msgs::LaserScanConstPtr&);
	void Resample();
};

#endif /* COPK_ASSIGNMENT_4_INCLUDE_SENSORUPDATE_H_ */
