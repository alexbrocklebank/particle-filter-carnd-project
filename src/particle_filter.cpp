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
	num_particles = 10;

	// Set up Gaussian Distributions with random generator
	default_random_engine gen;
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
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
	//std::vector<double> weights(num_particles, 1.0);

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
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	// Make a Particle iterator
	std::vector<Particle>::iterator it;

	for (it = particles.begin(); it != particles.end(); ++it) {
		double x = it->x;
		double y = it->y;
		double theta = it->theta;
		// If yaw rate is not Zero
		if (yaw_rate >= 0.000001) {
			it->x = x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
			it->y = y + (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
			it->theta = theta + (yaw_rate * delta_t);
		}
		// If yaw rate is Zero
		else {
			it->x = x + velocity * delta_t * cos(theta);
			it->y = y + velocity * delta_t * sin(theta);
			//it->theta = theta;
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
	std::cout << "NOT USED YET ......\n";
	
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
	
	// Loop through each Particle and convert 
	for (int p = 0; p < num_particles; p++) {
		double particle_x = particles[p].x;
		double particle_y = particles[p].y;
		double theta = particles[p].theta;

		// SetAssociations variables
		std:vector<int> associations;
		std::vector<double> x_observations;
		std::vector<double> y_observations;

		// Make a copy of the Map to remove entries from
		std::vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;

		std::cout << "Particle " << p << "\n";
		// Transform Observation to Map coordinates
		for (int cur_obs = 0; cur_obs < observations.size(); cur_obs++) {
			// Observation coordinates, X and Y
			double x_obs = observations[cur_obs].x;
			double y_obs = observations[cur_obs].y;

			// Landmarks Equation from Lesson 14.16 and Figure 3.33
			double x_map = particle_x + (cos(theta) * x_obs) - (sin(theta) * y_obs);
			double y_map = particle_y + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			std::cout << "Observation #" << cur_obs << " (" << x_map << ", " << y_map << ")\n";

			// Nearest Neighbor Algorithm
			// TODO: Make temp list of landmarks, and after each run, pop the selected landmark
			double min_distance = 1000.00;
			int closest_prediction = -1;
			Map::single_landmark_s nearest_neighbor;
			
			for (int cur_landmark = 0; cur_landmark < landmarks.size(); cur_landmark++) {
				double x_pred = landmarks[cur_landmark].x_f;
				double y_pred = landmarks[cur_landmark].y_f;
				if ((abs(x_map - x_pred) <= sensor_range) && (abs(y_map - y_pred) <= sensor_range)) {
					// Measure distance between points by Pythagorean Theorem
					// sqrt( ( x1 - x2 )**2 + ( y1 - y2 )**2 )
					double distance = pow((pow((x_map - x_pred), 2) + pow((y_map - y_pred), 2)), 0.5);
					if (distance < min_distance) {
						nearest_neighbor = map_landmarks.landmark_list[cur_landmark];
						closest_prediction = cur_landmark;
						min_distance = distance;
					}
					std::cout << "Landmark #" << cur_landmark << " (" << x_pred << ", " << y_pred << ")\n";
				}
			}
			// Associate Observations to Landmarks
			// Add coordinates converted to map space to sense data for the current particle
			associations.push_back(nearest_neighbor.id_i);
			x_observations.push_back(x_map);
			y_observations.push_back(y_map);
			particles[p] = SetAssociations(particles[p], associations, x_observations, y_observations);
			std::cout << "Particle " << p << " Associations Updated.\n";
			
			// Remove landmark from temp list, to reduce loop size each run and improve speed
			landmarks.erase(closest_prediction);

			// TODO: Where does below code belong?
			// Multivariate-Gaussian Probability, Lesson 14:19
			double sig_x = std_landmark[0];
			//std::cout << "sig_x = " << sig_x << "\n";
			double sig_y = std_landmark[1];
			//std::cout << "sig_y = " << sig_y << "\n";
			double mu_x = nearest_neighbor.x_f;
			//std::cout << "mu_x = " << mu_x << "\n";
			double mu_y = nearest_neighbor.y_f;
			//std::cout << "mu_y = " << mu_y << "\n";

			// TODO: Loop through Associations?
			double gauss_norm = (1.0 / (2.0 * M_PI * sig_x * sig_y));
			//std::cout << "gauss_norm = " << gauss_norm << "\n";
			double exponent = (pow((x_map - mu_x), 2.0)) / (2.0 * pow(sig_x, 2.0)) + (pow((y_map - mu_y), 2.0)) / (2.0 * pow(sig_y, 2.0)); 
			//std::cout << "exponent = " << exponent << "\n";
			double particle_weight = gauss_norm * exp(-exponent);
			//std::cout << "weight = " << weight << "\n";

			// Update particle weights and weights vector
			particles[p].weight = particle_weight;
			std::cout << "Particle " << p << ": \n";
			std::cout << "x:      " << particles[p].x << "\n";
			std::cout << "y:      " << particles[p].y << "\n";
			std::cout << "theta:  " << particles[p].theta << "\n";
			std::cout << "weight: " << particles[p].weight << "\n";
			//weights[particles[p].id] = weight;
			//std::cout << "Weights Updated.\n";
		}
	}

	std::cout << "Particle Filter Updating Weights Complete.\n";
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::cout << "Particle Filter Resampling......\n";

	// Variables
	std::vector<Particle> new_p;
	default_random_engine gen(time(0));
	uniform_real_distribution<double> dist(0.0, 1.0);
	int index = (int)(dist(gen) * num_particles);
	double beta = 0.0;
	double max_w = 0.0;

	// Find max weight in Particles
	for (int i = 0; i < num_particles; i++) {
		if (particles[i].weight > max_w) {
			max_w = particles[i].weight;
		}
	}
	// Resampling Wheel method from Lesson 13.20
	for (int i = 0; i < num_particles; i++) {
		beta += dist(gen) * 2.0 * max_w;
		while (beta > particles[index].weight) {
			beta -= particles[index].weight;
			index = (index + 1) % num_particles;
		}
		new_p.push_back(particles[index]);
	}

	particles = new_p;

	std::cout << "Particle Filter Resampling Complete.\n";
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

	return particle;
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
