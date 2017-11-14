/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *      Author: Yi-Ching Chung
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

// Default random engine
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.

  num_particles = 10;

  normal_distribution<double> dist_x(0, std[0]);
  normal_distribution<double> dist_y(0, std[1]);
  normal_distribution<double> dist_theta(0, std[2]);

  for (int i=0; i < num_particles; i++){

    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;
    
    // Add noise
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);

    // Add element at the end    
    particles.push_back(p);
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	// http://www.cplusplus.com/reference/random/default_random_engine/
  default_random_engine gen;
  
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i=0; i < num_particles; i++){
    // Calculate new state, particles[i] 
    if (fabs(yaw_rate) > 0.0001) {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate* delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta +=  yaw_rate * delta_t;
    }
    else{
      // Theta is unchanged with a zero yaw rate
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    // Add noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
  
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.
	// This is a helper during the updateWeights phase.
  
  //When size is going to be larger than 2^31 (2 billions)
  // Loop over observations vector
  for (unsigned int i = 0; i < observations.size(); i++) {

    // Get current observation
    LandmarkObs o = observations[i];

    // Initialize minimum distance to maximum possible. No negative values for distance
    double min_dist = numeric_limits<double>::max();

    // Initialize ID of landmark from map placeholder
    int map_id = -1;

    // Loop over predictions vector
    for (unsigned int j = 0; j < predicted.size(); j++) {

      // Get current predictions
      LandmarkObs p = predicted[j];
      
      // Assign distance between current/predicted landmarks by considering circular region around a particle
      double cur_dist = dist(o.x, o.y, p.x, p.y);
      
      // Data Association (Nearest Neighbor)
      // Find the predicted landmark nearest the current observed landmark
      // Assign the associated landmark's ID to that of particle
      if (cur_dist < min_dist) {
	min_dist = cur_dist;
	map_id = p.id;
      }
      
      // Set the observation's id to the nearest predicted landmark's id
      observations[i].id = map_id;
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution.
	// https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. The particles are located
	//   according to the MAP'S coordinate system then being transformed.
	//   This transformation requires both rotation AND translation (but no scaling).
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   The actual equation is from : equation 3.33 in
	//   http://planning.cs.uiuc.edu/node99.html
  
  // Loop over particles
  for (int i = 0; i < num_particles; i++) {
    //The "Update" step is to update particle weights based on LIDAR and RADAR readings of landmarks 

    // Get the particles' coordinates
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    // Placeholder for the map landmark locations (appearing in data association function)
    vector<LandmarkObs> predictions;

    // Loop over map landmarks
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // Get the ID and coordinates
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;

      // Consider landmarks only within sensor range of the particle by considering a rectangeular region
      if (fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range) {

	// Add predictions at the end
	predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
      }
    }

    // Transofrmed observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> transformed_os;

    // Loop over observations
    for (unsigned int j = 0; j < observations.size(); j++) {
      double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
      // Add transformation at the end
      transformed_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
    }

    // Associate the predictions and transformed observations on the particle
    dataAssociation(predictions, transformed_os);

    // Re-initialize weight coefficients
    particles[i].weight = 1.0;

    // Loop over projected observations 
    for (unsigned int j = 0; j < transformed_os.size(); j++) {

      // Create placeholders for observation and associated prediction coordinates
      double o_x, o_y, pr_x, pr_y;
      o_x = transformed_os[j].x;
      o_y = transformed_os[j].y;

      int associated_prediction = transformed_os[j].id;

      // Get the coordinates of the prediction associated with the current observation
      // Loop over predictions
      for (unsigned int k = 0; k < predictions.size(); k++) {
	if (predictions[k].id == associated_prediction) {
	  pr_x = predictions[k].x;
	  pr_y = predictions[k].y;
	}
      }

      // Calculate weight for this observation with multivariate Gaussian (normalization and exponent term)
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      double gauss_norm = (1/(2 * M_PI * s_x * s_y));
      double exponent  = exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );
      double obs_w = gauss_norm * exponent; 
      //double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

      // Get the particle's final weight based on all the calculated measurement probabilities
      particles[i].weight *= obs_w;
    }
  }
}


void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// More about std::discrete_distribution is in:
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<Particle> new_particles;

  vector<double> weights;

  // Loop over particles and get current weights
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // Update particles by using a discrete distribution function to Bayesian posterior distribution
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  // Get maximum weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // Implement uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // Resampling from 0 to length of particle array
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    // Resampling Wheel: Drawing a particle protitional to its weight
    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  //Assign reampled particles to current particles
  particles = new_particles;
}

void ParticleFilter::write(std::string filename) {
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  for (int i = 0; i < num_particles; ++i) {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
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
