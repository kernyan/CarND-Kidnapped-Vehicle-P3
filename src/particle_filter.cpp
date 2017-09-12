/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *      Revised by kernyan for SDC Term 2 Project Particle Filter
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
#include "assert.h"

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double mu_x, double mu_y, double theta, double std[]) {

  // initialize particles ~ p(x_0)
  // 1. each state is randomly generated from Norm (mu, sigma)
  // 2. created particle added to vector<particle>

  num_particles_ = 100;
  default_random_engine gen;
  normal_distribution<double> NDist_x(mu_x, std[0]);
  normal_distribution<double> NDist_y(mu_y, std[1]);
  normal_distribution<double> NDist_theta(theta, std[2]);

  for (int i = 0; i < num_particles_; ++i){
    Particle P;
    P.id_ = i;
    P.x_ = NDist_x(gen);
    P.y_ = NDist_y(gen);
    P.theta_ = NDist_theta(gen);
    P.weight_ = 1;

    particles_.push_back(P);
    weights_.push_back(P.weight_);
  }

  is_initialized_ = true;
}


void ParticleFilter::predict_without_noise(double delta_t, double velocity, double yaw_rate){

  // x_bar(t) ~ p( x_t | u_t, x_{t-1} )

  // move forward each particle to next time step

  for (auto && each : particles_){
    if (fabs(yaw_rate) > 0.001){
      each.x_ += (velocity/yaw_rate) * ( sin(each.theta_ + delta_t * yaw_rate) - sin(each.theta_));
      each.y_ += (velocity/yaw_rate) * (-cos(each.theta_ + delta_t * yaw_rate) + cos(each.theta_));
      each.theta_ += delta_t * yaw_rate;
    } else {
      each.x_ += delta_t * velocity * cos(each.theta_);
      each.y_ += delta_t * velocity * sin(each.theta_);
    }
  }
}


void ParticleFilter::add_prediction_noise(double std_pos[]){

  // x_bar(t) ~ p( x_t | u_t, x_{t-1} )

  // First apply deterministic prediction function on x_{t-1},
  // then draw new sample as new particle using old particle's state as mean and position sigma as variance  

  default_random_engine gen;

  for (auto && each : particles_){
    normal_distribution<double> NDist_x(each.x_, std_pos[0]);
    normal_distribution<double> NDist_y(each.y_, std_pos[1]);
    normal_distribution<double> NDist_theta(each.theta_, std_pos[2]);

    each.x_ = NDist_x(gen);
    each.y_ = NDist_y(gen);
    each.theta_ = NDist_theta(gen);
  }
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // line 4 of Table 4.3 in Thrun, Sebastian, et al. Probabilistic Robotics. MIT Press, 2010.

  // Drawing random samples from x_bar(t), where
  // x_bar(t) ~ p( x_t | u_t, x_{t-1} )

  predict_without_noise(delta_t, velocity, yaw_rate);
  add_prediction_noise(std_pos);
}


LandmarkObs ParticleFilter::Transform_Vehicle_To_Map(const Particle &P_in,
    const LandmarkObs &Obs) const {

  // observations obtained is relative to vehicle's coordinate
  // converting to global map coordinate

  LandmarkObs Out;
  double cosP = cos(P_in.theta_);
  double sinP = sin(P_in.theta_);
  Out.x = P_in.x_ + Obs.x * cosP - Obs.y * sinP;
  Out.y = P_in.y_ + Obs.x * sinP + Obs.y * cosP;
  return Out;
}


int ParticleFilter::ClosestLandmark(double sensor_range, const LandmarkObs &Trans_Obs, const Map &map_in) const {

  // for each observed landmark, we compare its distance to landmarks on global map
  // the global landmark whose distance is closest to the observed landmark
  // is selected as the most probable landmark

  double min_distance = sensor_range;
  double distance = 0;
  int min_id = -1;
  for (const auto &each_landmark : map_in.landmark_list){
    distance = dist(Trans_Obs.x, Trans_Obs.y, each_landmark.x_f, each_landmark.y_f);
    if (min_distance > distance){
      min_distance = distance;
      min_id = each_landmark.id_i;
    }
  }

  assert(min_id != -1);
  return min_id;
}


double ParticleFilter::NormProbWithoutConstant(const LandmarkObs &Trans_Obs,
                                 const Map &map_in,
                         double std_landmark[]) const {

  // finding the likelihood of an observation given current particle state

  // p( z_t | x_t )
  // since we only need the relative weights of all particles, we can drop the pdf's constant
  // here because x and y are both independent normal, their joint likelihood is given by their
  // product of their individual gaussian distribution

  // the gaussian distribution would have a mean of (distance from particle to landmark) and sigma of sensor noise

  const auto &L_list = map_in.landmark_list;
  int id = Trans_Obs.id;
  auto BestLandmark = find_if(L_list.begin(), L_list.end(),
                                 [id](const Map::single_landmark_s &lm)->bool{return lm.id_i == id;});

  double diff_x = Trans_Obs.x - (*BestLandmark).x_f;
  double diff_y = Trans_Obs.y - (*BestLandmark).y_f;
  double sig_x  = std_landmark[0];
  double sig_y  = std_landmark[1];
  return exp(-0.5*((diff_x * diff_x)/(sig_x * sig_x) + (diff_y * diff_y)/(sig_y * sig_y)));
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  // line 5 of Table 4.3 in Thrun, Sebastian, et al. Probabilistic Robotics. MIT Press, 2010.

  // update weights for each particle where,
  // weights = product of likelihood of each observations given state of particle

  LandmarkObs ObsInMapSpace;
  short i = 0;

  for (const auto &each_p : particles_){
    double Likelihood = 1.0;
    for (const auto &each_obs : observations){
      ObsInMapSpace    = Transform_Vehicle_To_Map(each_p, each_obs);
      ObsInMapSpace.id = ClosestLandmark(sensor_range, ObsInMapSpace, map_landmarks);
      Likelihood      *= NormProbWithoutConstant(ObsInMapSpace, map_landmarks, std_landmark);
    }

    weights_[i] = Likelihood;
    ++i;
  }
}


void ParticleFilter::PerformAssociation(Particle &P_in,
                    const std::vector<LandmarkObs> &observations,
                    const Map &map_in) const {

  // associate best particle with the observed landmarks
  // used for diagnostic purpose

  LandmarkObs Trans_Obs;
  double min_distance = 30;
  double distance = 0;
  int min_id;
  double x, y;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;

  for (const auto &each_landmark : map_in.landmark_list){
    min_id = -1;
    x = 0;
    y = 0;
    for (const auto &each_obs : observations){
      Trans_Obs    = Transform_Vehicle_To_Map(P_in, each_obs);
      distance = dist(Trans_Obs.x, Trans_Obs.y, each_landmark.x_f, each_landmark.y_f);
      if (min_distance > distance){
        min_distance = distance;
        min_id = each_landmark.id_i;
        x = Trans_Obs.x;
        y = Trans_Obs.y;
      }
    }

    if (min_id != -1 &&  distance < ASSOC_THRES){
      associations.push_back(min_id);
      sense_x.push_back(x);
      sense_y.push_back(y);
    }
  }

  SetAssociations(P_in, associations, sense_x, sense_y);
}


void ParticleFilter::resample() {

  // line 8 to 12 of Table 4.3 in Thrun, Sebastian, et al. Probabilistic Robotics. MIT Press, 2010.

  // each particle now has a predicted motion state, our particles collection now has a distribution of
  // p( x_{0:t} | z_{1:t-1}, u_{1:t} )

  // we want to update the distribution so that our particles collection has the following distribution
  // p( x_{0:t} | z_{1:t}, u_{1:t} )
  // in essense, we apply the weights of p(z_t | x_t) to our particles and resample according to these weights

  default_random_engine gen;
  discrete_distribution<int> DiscreteDist(weights_.begin(), weights_.end());

  vector<Particle> resampled;
  for (int i = 0; i < num_particles_; ++i){
    resampled.push_back(particles_[DiscreteDist(gen)]);
  }  

  particles_ = resampled;
}


Particle ParticleFilter::SetAssociations(Particle &particle,
    std::vector<int> associations,
    std::vector<double> sense_x,
    std::vector<double> sense_y) const
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations_.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations_ = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}


string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations_;
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
