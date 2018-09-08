/*
 * particle_filter.cpp
 */
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// This is a random number engine class that generates pseudo-random numbers.
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[])
{

	// init once per execution
	if (is_initialized)
		return;

	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// num of particle to consider in filter
	num_particles = 100;


	// create normal distribution array with starting value and hard set deviation/sigma
	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);

	// create a number of random particles around the initial position (x/y/theta)
	// within sigma
	for (int i = 0; i < num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;
		particles.push_back(particle);
		weights.push_back(1.0); // init weights to unity
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
		double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// get a set of random particles around the origin reference to vehicle
	// aka the reference position of the vehicle is (0,0,0) is the center of
	// its coord system
	std::normal_distribution<double> N_x(0, std_pos[0]);
	std::normal_distribution<double> N_y(0, std_pos[1]);
	std::normal_distribution<double> N_theta(0, std_pos[2]);

	// for particles used, predict its position
	for (int i = 0; i < num_particles; i++)
	{
		// if yaw is 0, just geometry
		if (yaw_rate == 0)
		{
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else   // consider yaw rate over time, turn speed
		{
			particles[i].x += (velocity / yaw_rate)
					* (sin(particles[i].theta + (yaw_rate * delta_t))
							- sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate)
					* (-1 * cos(particles[i].theta + (yaw_rate * delta_t))
							+ cos(particles[i].theta));

			particles[i].theta += (yaw_rate * delta_t);
		}

		// randomize the position a little within sensor accuracy
		particles[i].x += N_x(gen);
		particles[i].y += N_y(gen);
		particles[i].theta += N_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
		std::vector<LandmarkObs>& observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// for each observations and find the landmarks that are closest to it
	// considering the predicted landmarks were randomized
	//
	for (unsigned int i = 0; i < observations.size(); i++)
	{
		// get 1st observation.
		int min_landmark_id = -1;
		// distance to landmark
		double min_dist = numeric_limits<double>::max();

		// search the predicted landmark locations and get the closest landmark
		// to the selected observation.
		for (unsigned int j = 0; j < predicted.size(); j++)
		{
			double diff_x = observations[i].x - predicted[j].x;
			double diff_y = observations[i].y - predicted[j].y;

			// dont need to do square root since squaring does the job.
			double dist = diff_x * diff_x + diff_y * diff_y;

			// I found one, assign the id to the observation.
			if (dist < min_dist)
			{
				min_dist = dist;
				min_landmark_id = predicted[j].id;
			}
		}

		// the landmark must be this observation.
		observations[i].id = min_landmark_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// get sigmas
	double std_landmark_range = std_landmark[0];
	double std_landmark_bearing = std_landmark[1];

	// for each particle, cnonvert them from vehicle cooridnate system to mapped coordinate
	// system
	for (int i = 0; i < num_particles; i++)
	{
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// convert all observations in map coordinate system
		std::vector<LandmarkObs> mapped_obs;
		for (unsigned int j = 0; j < observations.size(); j++)
		{
			double xx = cos(theta) * observations[j].x
					- sin(theta) * observations[j].y + x;
			double yy = sin(theta) * observations[j].x
					+ cos(theta) * observations[j].y + y;
			mapped_obs.push_back(LandmarkObs
			{ observations[j].id, xx, yy });
		}

		// for each particle search the landmarks and see if we have a match
		// within range
		std::vector<LandmarkObs> pois;
		std::vector<Map::single_landmark_s> landmarks =
				map_landmarks.landmark_list;
		for (unsigned int j = 0; j < landmarks.size(); j++)
		{
			float land_x = landmarks[j].x_f;
			float land_y = landmarks[j].y_f;
			double diff_x = x - land_x;
			double diff_y = y - land_y;

			if ((diff_x * diff_x + diff_y * diff_y)
					< (sensor_range * sensor_range))
			{
				pois.push_back(LandmarkObs
				{ map_landmarks.landmark_list[j].id_i, land_x, land_y });
			}
		}

		// we have the landmarks close to the observations
		// in the right coordinate system, now associate
		dataAssociation(pois, mapped_obs);

		// set the weight to 1.0 and find the real weight
		particles[i].weight = 1.0;

		double land_x, land_y;

		// set the weights for the new observations based on the distance to the landmarks in mapped-coordinates
		for (unsigned int j = 0; j < mapped_obs.size(); j++)
		{
			// find closest landmark
			for (unsigned int u = 0; u < pois.size(); u++)
			{
				if (pois[u].id == mapped_obs[j].id)
				{
					land_x = pois[u].x;
					land_y = pois[u].y;
					break;
				}
			}

			// get xy diff distance
			double d_x = mapped_obs[j].x - land_x;
			double d_y = mapped_obs[j].y - land_y;

			// calculate weight via distance
			double std_bearing_range_2 = std_landmark_range
					* std_landmark_bearing;
			double std_bearing_2 = std_landmark_bearing * std_landmark_bearing;
			double std_range_2 = std_landmark_range * std_landmark_range;

			double prefix = 1.0 / (2 * M_PI * std_bearing_range_2);
			double exp_super = ((d_x * d_x) / (2 * std_range_2))
					+ ((d_y * d_y) / (2 * std_bearing_2));
			double weight = prefix * exp(-1 * exp_super);

			if (weight == 0) // or e-20++
			{
				// set it to something small.
				particles[i].weight *= 0.0001; // 0.00001
			}
			else
			{
				// multiply weight probablity into previous observation
				particles[i].weight *= weight; // 0.00001
			}
		}
	}
}

#ifdef XXXX
void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Get weights and max weight in the set of particles so we can determined if they need to be thrown out..
	vector<double> weights;
	double maxWeight = numeric_limits<double>::min();
	for (int i = 0; i < num_particles; i++)
	{
		weights.push_back(particles[i].weight);
		if (particles[i].weight > maxWeight)
		{
			maxWeight = particles[i].weight;
		}
	}

	// Creating distributions.
	uniform_real_distribution<double> distDouble(0.0, maxWeight);
	uniform_int_distribution<int> distInt(0, num_particles - 1);

	// Generating index.
	int index = distInt(gen);

	double beta = 0.0;

	// the wheel
	vector<Particle> resampledParticles;
	for (int i = 0; i < num_particles; i++)
	{
		beta += distDouble(gen) * 2.0;
		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resampledParticles.push_back(particles[index]);
	}

	particles = resampledParticles;
}
#endif

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<double> weights;
	for (int i = 0; i < num_particles; i++)
	{
		weights.push_back(particles[i].weight);
	}
	double beta = 0.0;

	auto max_weight = *std::max_element(weights.begin(), weights.end());

	uniform_real_distribution<double> random_weight(0.0, max_weight);
	uniform_int_distribution<int> particle_index(0, num_particles - 1);
	int current_index = particle_index(gen);

	vector<Particle> resampled;
	for (unsigned int i = 0; i < num_particles; i++)
	{
		beta += random_weight(gen);

		while (beta > weights[current_index])
		{
			beta -= weights[current_index];
			current_index = (current_index + 1) % num_particles;
		}
		resampled.push_back(particles[current_index]);
	}
	particles = resampled;
}

Particle ParticleFilter::SetAssociations(Particle& particle,
		const std::vector<int>& associations,
		const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
