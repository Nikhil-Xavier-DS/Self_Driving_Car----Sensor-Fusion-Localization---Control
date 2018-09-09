/*
 * Nikhil Xavier
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
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std_sigma[]) 
{
	normal_distribution<double> theta_length(theta, std_sigma[2]);
	normal_distribution<double> y_length(y, std_sigma[1]);
	normal_distribution<double> x_length(x, std_sigma[0]);
	
	int index = 0;
	float one = 1.0;
	for (index = 0; index < 100; ++index)
	{
		Particle particle;
		particle.id = index;
		particle.weight = one;
		particle.theta = theta_length(gen);
		particle.y = y_length(gen);		
		particle.x = x_length(gen);
		particles.push_back(particle);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double small_t, double std_sigma[], double velocity, double change_in_yaw) 
{
	int index = 0;
	for (index = 0; index < 100; ++index)
	{
		double past_theta = particles[index].theta;
		double past_y = particles[index].y;
		double past_x = particles[index].x;
		double future_theta;
		double future_y;
		double future_x;
		if (abs(change_in_yaw)>0.00001)
		{
			future_y = past_y + velocity/change_in_yaw*(cos(past_theta)-cos(future_theta));
			future_theta = past_theta + change_in_yaw*small_t;
			future_x = past_x + velocity/change_in_yaw*(sin(future_theta)-sin(past_theta));			
		}
		else
		{
			future_y = past_y + velocity*small_t*sin(past_theta);
			future_theta = past_theta;
			future_x = past_x + velocity*small_t*cos(past_theta);	
		}
		normal_distribution<double> theta_length(future_theta, std_sigma[2]);
		normal_distribution<double>  y_length(future_y, std_sigma[1]);
		normal_distribution<double> x_length(future_x, std_sigma[0]);
		particles[index].theta = theta_length(gen);
		particles[index].y = y_length(gen);
		particles[index].x = x_length(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmark_predict, std::vector<LandmarkObs>& landmark_obs) 
{
	for (auto& observation : landmark_obs)
	{
		double dist_least = 0;
		dist_least = numeric_limits<double>::max();
		for (const auto& observation_predicted : landmark_predict)
		{
			double dummy = 0;
			dummy = dist(observation.x, observation.y, observation_predicted.x, observation_predicted.y);
			if (dummy < dist_least)
			{
				observation.id = observation_predicted.id;
				dist_least = dummy;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) 
{
	double std_y = std_landmark[1];
	double std_x = std_landmark[0];
	int index = 0;
	for (index = 0; index<100; ++index)
	{
		double particle_theta = particles[index].theta;
		double particle_y = particles[index].y;
		double particle_x = particles[index].x;
		vector<LandmarkObs> landmark_future;
		for (const auto& landmark_map: map_landmarks.landmark_list)
		{
			int id_landmark = landmark_map.id_i;
			double y_landmark = (double)landmark_map.y_f;
			double x_landmark = (double)landmark_map.x_f;
			double dummy = dist(particle_x, particle_y, x_landmark, y_landmark);
			if (dummy < sensor_range)
			{
				LandmarkObs future_landmark;
				future_landmark.id = id_landmark;
				future_landmark.y = y_landmark;
				future_landmark.x = x_landmark;
				landmark_future.push_back(future_landmark);
			}
		}
		vector<LandmarkObs> reference_observed_map;
		int index1 = 0;
		for (index1 = 0; index1 < observations.size(); ++index1)
		{
			LandmarkObs observation_rotated;
			observation_rotated.y = sin(particle_theta)*observations[index1].x + cos(particle_theta)*observations[index1].y + particle_y;
			observation_rotated.x = cos(particle_theta)*observations[index1].x - sin(particle_theta)*observations[index1].y + particle_x;
			reference_observed_map.push_back(observation_rotated);
		}
		dataAssociation(landmark_future, reference_observed_map);
		double p_like = 1;
		double x_mu = 0;
		double y_mu = 0;
		for (const auto& dummy_map : reference_observed_map)
		{
			for (const auto& dummy_landmark: landmark_future)
			{
				if (dummy_map.id == dummy_landmark.id)
				{
					x_mu = dummy_landmark.x;
					y_mu = dummy_landmark.y;
					break;
				}
			}
			double factor_normalized = 8*M_PI*std_x*std_y/4;
			double dummy_probability = exp( -( pow(dummy_map.x - x_mu, 2) / (2 * std_x * std_x) + pow(dummy_map.y - y_mu, 2) / (2 * std_y * std_y) ) );
			p_like = p_like*dummy_probability/factor_normalized;
		}
		particles[index].weight = p_like;
	}
	double factor_normalized = 0;
	for (const auto& dummy : particles)
	{
		factor_normalized = factor_normalized + dummy.weight;
	}
	for (auto& dummy : particles)
	{
		dummy.weight = dummy.weight/(factor_normalized + numeric_limits<double>::epsilon());
	}	
	
}



void ParticleFilter::resample() 
{
    vector<double> dummy_particle;
    for (const auto& particle : particles)
	{
        dummy_particle.push_back(particle.weight);
	}
   
    discrete_distribution<int> weighted_distribution(dummy_particle.begin(), dummy_particle.end());
    vector<Particle> second_particles;
    int index = 0;
    for (index = 0; index < 100; ++index) 
	{
        	int dummy = weighted_distribution(gen);
        	second_particles.push_back(particles[dummy]);
   	 }
    particles = second_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
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
    s = s.substr(0, s.length()-1);
    return s;
}
string ParticleFilter::getSenseX(Particle best) 
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);
    return s;
}
