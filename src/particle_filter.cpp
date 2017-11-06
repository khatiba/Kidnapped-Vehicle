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
  num_particles = 10;
  weights.assign(num_particles, 1.0);

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

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  default_random_engine gen;

  for (int i = 0; i < num_particles; i++) {
    Particle p = particles[i];
    double xf;
    double yf;
    double thetaf;

    if (fabs(yaw_rate) < 0.001) {
      xf = p.x + velocity*delta_t*cos(p.theta);
      yf = p.y + velocity*delta_t*sin(p.theta);
      thetaf = p.theta;
    } else {
      xf = p.x + (velocity/yaw_rate)*(sin(p.theta + yaw_rate*delta_t) - sin(p.theta));
      yf = p.y + (velocity/yaw_rate)*(cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
      thetaf = p.theta + yaw_rate*delta_t;
    }

    double std_x     = std_pos[0];
    double std_y     = std_pos[1];
    double std_theta = std_pos[2];

    normal_distribution<double> noise_x(0, std_x);
    normal_distribution<double> noise_y(0, std_y);
    normal_distribution<double> noise_theta(0, std_theta);

    particles[i].x = xf + noise_x(gen);
    particles[i].y = yf + noise_y(gen);
    particles[i].theta = thetaf + noise_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  for (int i = 0; i < observations.size(); i++) {
    LandmarkObs obs = observations[i];

    int minId = 0;
    double minDist = numeric_limits<double>::max();
    for (int j = 0; j < predicted.size(); j++) {
      LandmarkObs pred = predicted[j];

      double newDist = dist(pred.x, pred.y, obs.x, obs.y);
      if (newDist < minDist) {
        minDist = newDist;
        minId = pred.id;
      }
    }
    observations[i].id = minId;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map) {

  double sigx = std_landmark[0];
  double sigy = std_landmark[1];

  for (int i = 0; i < num_particles; i++) {
    Particle particle = particles[i];

    vector<LandmarkObs> predicted;
    for (int j = 0; j < map.landmark_list.size(); j++) {
      Map::single_landmark_s landmark = map.landmark_list[j];
      double lpdist = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
      if (lpdist < sensor_range) {
        predicted.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
      }
    }

    vector<LandmarkObs> actual;
    for (int j = 0; j < observations.size(); j++) {
      LandmarkObs obs = observations[j];
      double xm = particle.x + cos(particle.theta)*obs.x - sin(particle.theta)*obs.y;
      double ym = particle.y + sin(particle.theta)*obs.x + cos(particle.theta)*obs.y;
      actual.push_back(LandmarkObs{obs.id, xm, ym});
    }

    dataAssociation(predicted, actual);

    weights[i] = 1.0;

    for (int j = 0; j < actual.size(); j++) {
      LandmarkObs obs = actual[j];

      double lx = 0.0;
      double ly = 0.0;
      for (int k = 0; k < predicted.size(); k++) {
        LandmarkObs pred = predicted[k];
        if (pred.id == obs.id) {
          lx = pred.x;
          ly = pred.y;
          break;
        }
      }

      double gaussNorm = 1/(2*M_PI*sigx*sigy);
      double weight = gaussNorm * exp(-(pow(obs.x-lx,2)/(2*pow(sigx,2)) + pow(obs.y-ly,2)/(2*pow(sigy,2))));
      weights[i] *= weight;
    }
  }
}

void ParticleFilter::resample() {
  default_random_engine gen;
  discrete_distribution<int> dist(weights.begin(), weights.end());

	std::vector<Particle> newParticles;
  for (int i = 0; i < num_particles; i++) {
    int index = dist(gen);
    newParticles.push_back(particles[index]);
  }

  particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, 
    std::vector<double> sense_x, std::vector<double> sense_y) {
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

