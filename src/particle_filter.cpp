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
  num_particles = 100;
  is_initialized = false;
  weights.assign(num_particles, 1);

  double std_x     = std[0];
  double std_y     = std[1];
  double std_theta = std[2];

  // Create normal distribution around the initial GPS coordinates
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

  default_random_engine gen;

  // Initialize particles randomly around the normal distribution
  for (int i = 0; i < num_particles; i++) {
    double sample_x     = dist_x(gen);
    double sample_y     = dist_y(gen);
    double sample_theta = dist_theta(gen);

    Particle particle;
    particle.id = i;
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    particle.weight = weights[i];
    particles.push_back(particle);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  for (int i = 0; i < num_particles; i++) {
    Particle p = particles[i];
    double xf;
    double yf;
    double thetaf;

    if (abs(yaw_rate) < 0.001) {
      xf = p.x + velocity*delta_t*cos(p.theta);
      yf = p.y + velocity*delta_t*sin(p.theta);
    } else {
      xf = p.x + (velocity/yaw_rate)*(sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      yf = p.y + (velocity/yaw_rate)*(cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
    }
    thtetaf = p.theta + yaw_rate*delta_t;

    double std_x     = std_pos[0];
    double std_y     = std_pos[1];
    double std_theta = std_pos[2];

    normal_distribution<double> noise_x(x, std_x);
    normal_distribution<double> noise_y(y, std_y);
    normal_distribution<double> noise_theta(theta, std_theta);

    default_random_engine gen;

    p.x = xf + noise_x(gen);
    p.y = yf + noise_y(gen);
    p.theta = thetaf + noise_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
