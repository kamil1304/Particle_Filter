/*
 * SensorUpdate.cpp
 *
 *  Created on: 5 Dec 2015
 *      Author: Konrad Cop
 *
 *  The class which perform the whole sensor update step, it assigns the weight to every particle and do the resampling afterwards
 */

#include "SensorUpdate.h"

SensorUpdate::SensorUpdate( Map2D* m, Parameters* p, Particles* part)
{
	particles=part;
	params=p;
	map=m;
}

SensorUpdate::~SensorUpdate() {
	// TODO Auto-generated destructor stub
}

// repeating the procedure in the loop
void SensorUpdate::callback(const sensor_msgs::LaserScanConstPtr& scan)
{
	while(!(params->GetStage()) && ros::ok())
	{
		ComputeWeight(scan); // computing the weights of every particle

		// if the robot moves, and the sum of weights non-zero (in order to not divide by 0) the particles are resampled
		if(particles->GetCurrentPose().pose.position.x!=particles->GetPreviousPose().pose.position.x)
		{
			if (particles->sum_of_weights>0)
			{
				Resample(); //resampling procedure
			}
		}
		particles->DetermineLocalizedPose();
		// switch the stage - motion update is activated, sensor update waits
		params->SetStage(true);
	}
}

// Resampling using roulette resampling method
void SensorUpdate::Resample()
{
	float random_number,weights_sum;
	int particle_index;

	for( int i=0; i<particles->particles.size(); i++)
	{
		random_number=0;
		weights_sum=0;

		// random number within range
		random_number=(float(rand()) / float(RAND_MAX))*params->rounding_factor;

		// resample procedure for every particle
		for(int j=0; j<=particles->particles.size(); j++)
		{
			weights_sum=weights_sum+particles->GetWeight(j);

			if(weights_sum>=random_number)
			{
				particle_index=j;
				break;
			}
		}
		particles->ReplaceParticle(i,particle_index);
	}
}

// procedure for computing the weight
void SensorUpdate::ComputeWeight(const sensor_msgs::LaserScanConstPtr& scan)
{
	double resolution=map->resolution;
	double width=resolution*map->width;
	double height=resolution*map->height;
	vector<int> local_map_occupancy; // the occupancy on the points visible by the laser
	vector<int> global_map_occupancy; // the occupancy on the corresponding points in the global map

	// the loop over every particle
	for(int i=0; i<params->number_of_particles; i++)
	{
		local_map_occupancy.clear();
		global_map_occupancy.clear();

		bool in_progress=true;
		bool out_of_map=false;
		int no_occupancy=0;
		int corr_points=0;
		int out_of_map_counter=0;
		int it=scan->ranges.size()-1; // iterator for the laser scanner
		int max_left=scan->ranges.size()/2-1; // maximum number of the laser beam
		float weight=0;

		// the loop over every scan for each particle
		while(in_progress)
		{
			double temp_x, temp_y; // coordinates of the points, at the end of the laser beam

			// -----------determination of the position of the laser point in the global map--------
			// considering only the points within the laser range
			if(scan->ranges.at(it)<=scan->range_max)
			{
				// coordinates in the global map
				temp_x=particles->particles.at(i).GetPose().x+scan->ranges.at(it)*cos((it-max_left)*scan->angle_increment+particles->particles.at(i).GetPose().theta);
				temp_y=particles->particles.at(i).GetPose().y+scan->ranges.at(it)*sin((it-max_left)*scan->angle_increment+particles->particles.at(i).GetPose().theta);

				// rounding to the resolution of the map (comparison purposes)
				temp_x=round(temp_x * (1/resolution)) * resolution;
				temp_y=round(temp_y * (1/resolution)) * resolution;
			}

			// checking if the detected point is within the map
			if ((temp_x<width && temp_x>=map->origin_x) && (temp_y<height && temp_y>=map->origin_y))
			{
				local_map_occupancy.push_back(params->occupancy_value);
				global_map_occupancy.push_back(map->GetOccupancyAt(temp_x,temp_y));
			}
			else
			{
				out_of_map_counter++;
				local_map_occupancy.push_back(no_occupancy);
				global_map_occupancy.push_back(params->occupancy_value);

				// threshold for outliers (laser not ideal some points lying in the map may be detected as outliers)
				if(out_of_map_counter>params->outliers_threshold)
				{
					out_of_map=true;;
					break;
				}
			}

			it=it-params->scan_filter; // selection of not every laser beam

			if(it<0) // end of loop
			{
				in_progress=false;
			}
		}

		//----------- comparing global and local map -------------
		// eliminating points out of map
		if (out_of_map)
		{
			weight=0;
			out_of_map=false;
		}

		else
		{
			//checking if particle is outside the map (a particle not a point of the scanner)
			bool particle_outside=false;
			float x=floor(particles->particles.at(i).GetPose().x * (1/resolution)) * resolution;
			float y=floor(particles->particles.at(i).GetPose().y * (1/resolution)) * resolution;

			if (((particles->particles.at(i).GetPose().x>=width) or (particles->particles.at(i).GetPose().x<=map->origin_x)) or
					((particles->particles.at(i).GetPose().y>=height) or (particles->particles.at(i).GetPose().y<=map->origin_y)))
			{
				particle_outside=true;
			}

			// checking if particle is not at the obstacle
			else if((map->GetOccupancyAt(x,y)) ==params->occupancy_value)
			{
				particle_outside=true;
			}

			if(particle_outside)
			{
				weight=0;
				particle_outside=false;
			}
			else
			{
				for(int l=0; l<local_map_occupancy.size();l++)
				{
					if(local_map_occupancy.at(l)==global_map_occupancy.at(l)) corr_points++;
				}
				weight=float(corr_points)/float(local_map_occupancy.size());// assigning the weight
			}
		}

		// storing the weight of one particle in the vector
		if(weight>particles->GetMinimumWeight())
		{
			particles->AssignWeight(weight,i);
		}
		// if weight is really small - assign minimum weight
		else
		{
			particles->AssignWeight(particles->GetMinimumWeight(), i);
		}
	}

	//---------- normalization procedures -------
	particles->NormalizeWeights();
}
