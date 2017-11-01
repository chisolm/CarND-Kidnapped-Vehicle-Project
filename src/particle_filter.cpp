/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *  Modified on: Oct 14, 2017
 *      Author: Chris Chisolm
 */

#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[], double std_landmark[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 10;  // 20 or 40 does not significantly increase accuracy

    particles.resize(num_particles);
    weights.resize(num_particles);

    // This line creates a normal (Gaussian) distribution for x, y and theta
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    cout << "init " << x << ", " << y << ", " << theta << ", " << std << endl;
    for (int i=0; i < num_particles; i++) {
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;
        if (debug > 1) {
            cout << "Particle " << i << ": " << particles[i].x << " " << particles[i].y << " " <<
                    particles[i].theta << endl;
        }
    }

    // Precompute for later use.
    inv_2_pi_sigx_sigy = 1/(2 * M_PI * std_landmark[0] * std_landmark[1]);
    two_sig_x_sq = 2 * std_landmark[0] * std_landmark[0];
    two_sig_y_sq = 2 * std_landmark[0] * std_landmark[0];

    // Flag, if filter is initialized
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    double xf, yf, thetaf;
    double prediction_x = 0, prediction_y = 0, prediction_th = 0;
    double pred_adj_x = 0, pred_adj_y = 0, pred_adj_th = 0;

    std::normal_distribution<> norm_x(0.0, std_pos[0]);
    std::normal_distribution<> norm_y(0.0, std_pos[1]);
    std::normal_distribution<> norm_theta(0.0, std_pos[2]);

    if (fabs(yaw_rate) < .0001) {
        if (yaw_rate > 0) {
            yaw_rate = .0001;  // minimum reasonable found to be at least as low as .0001
        } else {
            yaw_rate = -.0001;
        }
    }

    for (int i=0; i < num_particles; i++) {
        xf = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
        yf = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        thetaf = particles[i].theta + yaw_rate * delta_t;

        // main.cpp calls this function with std_pos labeled as "GPS measurement uncertainty".
        // Unless this is the same as the prediction uncertainty, the update with this value
        // appears to be incorrect.  Leaving in as clearly intended by the function prototype.
        xf = xf + norm_x(gen);
        yf = yf + norm_y(gen);
        thetaf = thetaf + norm_theta(gen);

        if (debug > 0) {
            cout << "Prediction Particle " << i << ": " << xf << " " << yf << " " << thetaf << endl;
            pred_adj_x += fabs(particles[i].x - xf);
            pred_adj_y += fabs(particles[i].y - yf);
            pred_adj_th += fabs(particles[i].theta - thetaf);
            prediction_x += xf;
            prediction_y += yf;
            prediction_th += thetaf;
        }  // debug

        particles[i].x = xf;
        particles[i].y = yf;
        particles[i].theta = thetaf;
    }
    if (debug > 0) {
        pred_adj_x = pred_adj_x / num_particles;
        pred_adj_y = pred_adj_y / num_particles;
        pred_adj_th = pred_adj_th / num_particles;
        cout << "Prediction avg adj: " << pred_adj_x << "\t" << pred_adj_y << "\t" << pred_adj_th << endl;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    double closestsum, sum, x_diff, y_diff;
    int map_index = 0, map_id = 0;

    for (int i = 0; i < observations.size(); i++) {
        LandmarkObs obs = observations[i];

        closestsum = 1024;  // world sized maximum value, should define elsewhere
        map_index = 0;
        for (int j = 0; j < predicted.size(); j++) {
            LandmarkObs pred_lm = predicted[j];
            x_diff = (pred_lm.x - obs.x);
            y_diff = (pred_lm.y - obs.y);
            sum = x_diff * x_diff + y_diff * y_diff;
            if (sum < closestsum) {
                closestsum = sum;
                map_id = pred_lm.id;
                map_index = j;
            }
        }
        // This counts on the index of the transformed observations to be the same as the
        // original observations.
        observations[i].id = map_id;
        if (debug > 1) {
            cout << "Association obs " << i << " x " << obs.x << " " << obs.y <<
                    " pred " << map_id << " " << predicted[map_index].x <<
                    "  " << predicted[map_index].y << endl;
        }
    }
}

double ParticleFilter::multivariate_gaussian(double x_obs_map, double y_obs_map, double x_ref, double y_ref) {
    // diff of observation and closest(given) point
    double x_diff_2 = (x_obs_map - x_ref) * (x_obs_map - x_ref);
    double y_diff_2 = (y_obs_map - y_ref) * (y_obs_map - y_ref);
    double out;

    out = inv_2_pi_sigx_sigy * exp(-1 * (x_diff_2/(two_sig_x_sq) + y_diff_2/(two_sig_y_sq)));

    return out;
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

    cout << "Update count: " << updatecycle++ << endl;

    for (int i=0; i < num_particles; i++) {
        if (debug > 1) {
            cout << "Update Particle " << i << ": " << particles[i].x << " " << particles[i].y << " " <<
                    particles[i].theta << endl;
        }
        Particle& cpar = particles[i];

        // For each particple position, creat a list of landmarks within range
        vector<LandmarkObs> l_inrange;

        // Create a list of landmark within sensor range.
        // Approximate sensor range without the sqrt().
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            float x = map_landmarks.landmark_list[j].x_f;
            float y = map_landmarks.landmark_list[j].y_f;

            // Close enough, will include some that shouldn't be in range.
            // if (dist(x, y, p_x, p_y) < sensor_range)
            if (fabs(x - cpar.x) <= sensor_range && fabs(y - cpar.y) <= sensor_range) {
                l_inrange.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, x, y});
            }
        }

        // Translate observations into map coordinates
        vector<LandmarkObs> obs_in_mapcoord;
        for (int j=0; j < observations.size(); j++) {
            float x_obs_map, y_obs_map;

            LandmarkObs cobs = observations[j];
            // Translate observation to map coordinates
            x_obs_map = cpar.x + (cos(cpar.theta) * cobs.x) - (sin(cpar.theta) * cobs.y);
            y_obs_map = cpar.y + (sin(cpar.theta) * cobs.x) + (cos(cpar.theta) * cobs.y);
            obs_in_mapcoord.push_back(LandmarkObs{cobs.id, x_obs_map, y_obs_map});
        }

        dataAssociation(l_inrange, obs_in_mapcoord);
        if (debug > 1) {
            for (int j=0; j < obs_in_mapcoord.size(); j++) {
                cout << "obs_in_mapcoord " << j << " x " << obs_in_mapcoord[j].x << " " <<
                        obs_in_mapcoord[j].y << " id " << obs_in_mapcoord[j].id << endl;
            }
        }
        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;

        double w = 1.0;
        double dbg_avg_diff = 0;
        for (int j=0; j < obs_in_mapcoord.size(); j++) {
            // Now id is the landmark map id which is +1 of the index to map_landmarks.landmark_list
            // The following code depends on that relationship to avoid having to lookup from the
            // location id to the vector index.  If this mapping fails, the code will fail completely.
            int id = obs_in_mapcoord[j].id;
            double x = obs_in_mapcoord[j].x;
            double y = obs_in_mapcoord[j].y;
            associations.push_back(id);
            sense_x.push_back(x);
            sense_y.push_back(y);
            id -= 1;
            w *= multivariate_gaussian(x, y,
                                map_landmarks.landmark_list[id].x_f,
                                map_landmarks.landmark_list[id].y_f);
            if (debug > 0) {
                cout << "Weight diff: par " << x - map_landmarks.landmark_list[id].x_f << " " <<
                        y - map_landmarks.landmark_list[id].y_f << endl;
                cout << "Weight comp: par " << i << " obs " << j << ": " << x << " " <<
                        y << " w " << w << endl;
                dbg_avg_diff += fabs(x - map_landmarks.landmark_list[id].x_f);
                dbg_avg_diff += fabs(y - map_landmarks.landmark_list[id].y_f);
            }
        }
        if (debug > 0) {
            dbg_avg_diff = dbg_avg_diff / (2 * obs_in_mapcoord.size());
            cout << "Average diff: " << dbg_avg_diff << endl;
            cout << "Create associations: " << i << " s: " << associations.size() << " ";
            for (auto p : associations) {
                std::cout << p << " ";
            }
            std::cout << endl;
        }

        cpar = SetAssociations(cpar, associations, sense_x, sense_y);

        cpar.weight = w;
        weights[i] = w;
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    double total_weight = 0;

    default_random_engine gen;
    std::discrete_distribution<> d(weights.begin(), weights.end());

    std::vector<Particle> new_particles;
    new_particles.resize(num_particles);
    if (debug > 1) {
        for (int i=0; i < num_particles; i++) {
            cout << "Old Particle: " << i << " x: " << particles[i].x << " y: " << particles[i].y << " th: "
                    << particles[i].theta << " w: " << particles[i].weight << endl;
        }
        for (int i=0; i < num_particles; i++) {
            cout << "Old associations: " << i << " s: " << particles[i].associations.size() << " ";
            for (auto p : particles[i].associations) {
                std::cout << p << " ";
            }
            std::cout << endl;
        }
    }

    for (int i=0; i < num_particles; i++) {
        int n = d(gen);
        new_particles[i] = particles[n];
    }

    particles = new_particles;

    if (debug > 1) {
        for (int i=0; i < num_particles; i++) {
            cout << "New Particle assign: " << i << " x: " << particles[i].x << " y: " << particles[i].y << " th: " << particles[i ].theta << endl;
        }
        for (int i=0; i < num_particles; i++) {
            cout << "New associations: " << i << " s: " << particles[i].associations.size() << " ";
            for (auto p : particles[i].associations) {
                std::cout << p << " ";
            }
            std::cout << endl;
        }
    }
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    // Clear the previous associations
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
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
