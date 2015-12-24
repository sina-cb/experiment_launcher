#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>
#include <string>

#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <glog/logging.h>
#include "Sampler.h"
#include "Sample.h"
#include "MCFHMM.h"
#include "DETree.h"
#include "Observation.h"
#include "Timer.h"

using namespace std;

ros::Subscriber sub;
ros::Publisher marker_pub;

vector<double> pi_low_limits;
vector<double> pi_high_limits;
vector<double> m_low_limits;
vector<double> m_high_limits;
vector<double> v_low_limits;
vector<double> v_high_limits;
vector<Observation> obs;

void init_observations(size_t size){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    uniform_real_distribution<double> dist(-.1, .1);
    double x = 5;
    double y = 5;
    double z = 5;
    obs.clear();
    for (size_t i = 0; i < size / 2; i++){
        Observation temp1;
        temp1.values.push_back(.1 + dist(gen));
        temp1.values.push_back(.1 + dist(gen));
        temp1.values.push_back(.1 + dist(gen));
        obs.push_back(temp1);

        Observation temp2;
        temp2.values.push_back(x + dist(gen));
        temp2.values.push_back(y + dist(gen));
        temp2.values.push_back(z + dist(gen));
        obs.push_back(temp2);
    }
}

void init_limits(){
    pi_low_limits.push_back(0);

    pi_high_limits.push_back(1);

    m_low_limits.push_back(0);
    m_low_limits.push_back(0);

    m_high_limits.push_back(1);
    m_high_limits.push_back(1);

    v_low_limits.push_back(0);
    v_low_limits.push_back(0);
    v_low_limits.push_back(0);
    v_low_limits.push_back(0);

    v_high_limits.push_back(6);
    v_high_limits.push_back(6);
    v_high_limits.push_back(6);
    v_high_limits.push_back(1);
}


int hmm()
{
	init_limits();
    init_observations(100);

    MCFHMM hmm;

    int N = 1000;
    int max_iteration = 10;

    hmm.set_limits(&pi_low_limits, &pi_high_limits, &m_low_limits, &m_high_limits, &v_low_limits, &v_high_limits);

    // Learn the HMM structure
    {
        Timer tmr;
        double t1 = tmr.elapsed();
        hmm.learn_hmm(&obs, max_iteration, N);
        double t2 = tmr.elapsed();
        cout << "Learning Time: " << (t2 - t1) << " seconds" << endl;
    }

    init_observations(10);

    vector<vector<Sample> > forward = hmm.forward(&obs, 100);

    int tr = 0;
    for (size_t i = 1; i < forward.size(); i++){
        double state0 = 0.0;
        double state1 = 0.0;
        for (size_t j = 0; j < forward[i].size(); j++){
            if (forward[i][j].values[0] > 0.5){
                state1 += forward[i][j].p;
            }else{
                state0 += forward[i][j].p;
            }
        }

        ROS_INFO("Observation %d", (int)i);
        ROS_INFO("State 0: %f State 1: %f\n", state0, state1);

        if (i % 2 == 1){
            if (state0 > state1){
               tr++;
            }
        }else{
            if (state1 > state0){
               tr++;
            }
        }

//        vector<Sample> temp = forward[i];
//        DETree tree(temp, &pi_low_limits, &pi_high_limits);
//        Sample state;
//        state.values.push_back(0.5);
//        cout << "Density: " << tree.density_value(state, hmm._rho()) << endl;
    }

    ROS_INFO("True: %d", tr);

    // BRANCH

    return 0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_prediction_node");
	ros::NodeHandle n;

	// n.param("robot_name", robot_name, std::string("Not Available"));
	// n.param("frame_id", frame_id, std::string("no_frame_id"));
	// n.param("namespace_", namespace_, std::string("no_namespace"));

	// sub = n.subscribe("/gazebo/model_states", 1000, chatterCallback);
	// marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	hmm();

	ros::spin();
	return 0;
}

