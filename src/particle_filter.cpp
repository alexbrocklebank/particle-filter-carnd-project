/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	std::cout << "Particle Filter Initialization......\n";

	// Number of particles to draw
	num_particles = 1000;

	// Set up Gaussian Distributions with random generator
	default_random_engine gen;
	float std_x = std[0];
	float std_y = std[1];
	float std_theta = std[2];
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Create particles from distributions
	for (int i = 0; i < num_particles; i++) {
		Particle p = Particle();
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
	}

	// Vector of weights of all particles
	std::vector<double> weights(num_particles, 0.0);

	// Flag, if filter is initialized
	is_initialized = true;

	std::cout << "Particle Filter Initialization Complete.\n";
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::cout << "Particle Filter Prediction......\n";

	// Set up Gaussian Distributions with random generator
	default_random_engine gen;
	float std_x = std_pos[0];
	float std_y = std_pos[1];
	float std_theta = std_pos[2];

	// Make a Particle iterator
	std::vector<Particle>::iterator it;

	for (it = particles.begin(); it != particles.end(); ++it) {
		float x = it->x;
		float y = it->y;
		float theta = it->theta;
		// If yaw rate is not Zero
		if (yaw_rate >= 0.001) {
			it->x = x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
			it->y = y + (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
			it->theta = theta + (yaw_rate * delta_t);
		}
		// If yaw rate is Zero
		else {
			it->x = sin(theta) * velocity * delta_t;
			it->y = cos(theta) * velocity * delta_t;
			it->theta = theta;
		}
		normal_distribution<double> dist_x(it->x, std_x);
		normal_distribution<double> dist_y(it->y, std_y);
		normal_distribution<double> dist_theta(it->theta, std_theta);
		it->x = dist_x(gen);
		it->y = dist_y(gen);
		it->theta = dist_theta(gen);
	}
	
	std::cout << "Particle Filter Prediction Complete.\n";
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	std::cout << "Particle Filter Data Association......\n";

	std::cout << "Particle Filter Data Association Complete.\n";
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	std::cout << "Particle Filter Updating Weights......\n";

	std::cout << "Particle Filter Updating Weights Complete.\n";
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::cout << "Particle Filter Resampling......\n";

	std::cout << "Particle Filter Resampling Complete.\n";
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
