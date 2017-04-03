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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 1000;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dx(x,std[0]);
  std::normal_distribution<double> dy(y,std[1]);
  std::normal_distribution<double> dtheta(theta,std[2]);

  for (int i = 0; i<num_particles; i++)
  {
    Particle tPart = {i, dx(gen), dy(gen), dtheta(gen), 1.0}; // id, x, y, theta, weight
    particles.push_back (tPart);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  //std::random_device rd;
  //std::mt19937 gen(rd());
  //std::normal_distribution<double> dvelocity(velocity,std[1]);
  //std::normal_distribution<double> dtheta(yaw_rate,std[2]);

  for (int i = 0; i<num_particles; i++)
  {
    Particle tPart = particles[i]; // get particle
    tPart.theta = tPart.theta + yaw_rate;
    tPart.x = tPart.x + cos(tPart.theta)*velocity;
    tPart.y = tPart.y + cos(tPart.theta)*velocity;
    particles[i] = tPart; // set particle
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
      /**
       * updateWeights Updates the weights for each particle based on the likelihood of the
       *   observed measurements.
       * @param sensor_range Range [m] of sensor
       * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
       *   standard deviation of bearing [rad]]
       * @param observations Vector of landmark observations
       * @param map Map class containing map landmarks
       */

	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
  for (int i = 0; i<num_particles; i++)
  {
    Particle tPart = particles[i]; // get particle
    double px = tPart.x;
    double py = tPart.y;
    double ptheta = tPart.theta;
    std::vector<LandmarkObs> transfered_observations; // rotated and transfered observations to origin coord system
    int num_observations = observations.size();
    for (int i = 0; i < num_observations; i++)
    {
      LandmarkObs temp_observation;
      temp_observation.x = observations[i].x*cos(ptheta) - observations[i].y*sin(ptheta) + px;  // this is TODO if coordinate axis needs change
      temp_observation.y = observations[i].x*sin(ptheta) + observations[i].y*cos(ptheta) + py;  // this is TODO if coordinate axis needs change
      temp_observation.id = 0;
      // check to which landmark it corresponds
      double xdist = (temp_observation.x-map_landmarks.landmark_list[0].x_f);
      double ydist = (temp_observation.y-map_landmarks.landmark_list[0].y_f);
      int min_dist_id = map_landmarks.landmark_list[0].id_i;
      double min_dist = sqrt(xdist*xdist+ydist*ydist);
      for (int j = 1; j<map_landmarks.landmark_list.size(); j++)
      {
        xdist = (temp_observation.x-map_landmarks.landmark_list[j].x_f);
        ydist = (temp_observation.y-map_landmarks.landmark_list[j].y_f);
        double dist = sqrt(xdist*xdist+ydist*ydist);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_dist_id = map_landmarks.landmark_list[j].id_i;
        }
      }
      temp_observation.id = min_dist_id;
      transfered_observations.push_back(temp_observation);
    }
    // transferred observations to origin coordinate system and predicted closest landmark id (duplicate currently allowed)
    // TODO: calculate distance to landmarks and set the weights

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> ddist(0.0,std_landmark[0]);

    std::vector<double> distances;

    for (int i = 0; i < num_observations; i++)
    {
      double xdist = (transfered_observations[i].x-map_landmarks.landmark_list[transfered_observations[i].id].x_f);
      double ydist = (transfered_observations[i].y-map_landmarks.landmark_list[transfered_observations[i].id].y_f);
      double dist = sqrt(xdist*xdist+ydist*ydist);
      dist += ddist(gen);
      distances.push_back(dist);
    }

    double prob = 1.0;
    double sigma = std_landmark[0];
    for (int i = 0; i < num_observations; i++)
    {
      double xdist = (transfered_observations[i].x-map_landmarks.landmark_list[transfered_observations[i].id].x_f);
      double ydist = (transfered_observations[i].y-map_landmarks.landmark_list[transfered_observations[i].id].y_f);
      double dist = sqrt(xdist*xdist+ydist*ydist);
      double gauss = exp(-((dist - distances[i])*(dist - distances[i])) / (sigma*sigma) / 2.0) / sqrt(2.0 * M_PI * (sigma*sigma));
      prob *= gauss;
    }
    particles[i].weight = prob;

    /*
    Z = []
    for i in range(len(landmarks)):
      dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
      dist += random.gauss(0.0, self.sense_noise)
      Z.append(dist)
    measurement = Z


    prob = 1.0;
    for i in range(len(landmarks)):
      dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
      prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
      //     Gaussian(dist, self.sense_noise, measurement[i]) // mu, sigma, x
      //     Gaussian = exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    return prob
    */

    //for (int i = 0; i < num_observations; i++)
    //{
    //  double xdist = (transf_observations[i].x-map_landmarks.landmark_list[transf_observations[i].id].x_f);
    //  double ydist = (transf_observations[i].y-map_landmarks.landmark_list[transf_observations[i].id].y_f);
    //  double dist = sqrt(xdist*xdist+ydist*ydist);
    //
    //}
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::vector<Particle> new_particles;
  double max_weight = particles[0].weight;
  for (int i = 1; i<num_particles; i++)
  {
    if (particles[i].weight > max_weight) {
      max_weight = particles[i].weight;
    }
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> ddouble(0.0, max_weight);
  std::uniform_int_distribution<int> dint(0, num_particles);

  int idx = dint(gen);
  double beta = 0.0;

  for (int i = 1; i<num_particles; i++)
  {
    beta = beta + ddouble(gen)*2.0;
    while (beta > particles[idx].weight)
    {
      beta = beta - particles[idx].weight;
      idx = (idx+1)%num_particles;
    }
    new_particles.push_back(particles[idx]);
  }
  particles = new_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
