#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>
#include <string>

using namespace std;

string robot_name;
string frame_id;
string namespace_;
ros::Subscriber sub;
ros::Publisher marker_pub;

float r[] = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
float g[] = {0.0, 1.0, 0.0, 0.0, 1.0, 0.0};
float b[] = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0};

void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){

	for (int i = 0; i < msg->name.size(); i++){
		if (msg->name[i] == robot_name){
			visualization_msgs::Marker points;
			points.header.frame_id = frame_id.c_str();
			points.header.stamp = ros::Time::now();
			points.ns = namespace_.c_str();
			points.action = visualization_msgs::Marker::ADD;
			points.id = i;
			points.type = visualization_msgs::Marker::POINTS;

			points.scale.x = .5;
  			points.scale.y = .5;

  			points.color.r = r[i];
  			points.color.g = g[i];
  			points.color.b = b[i];
  			points.color.a = 1.0f;

  			points.pose.orientation.w = 1.0;

  			points.pose.position.x = .1;
  			points.pose.position.y = .2;

  			geometry_msgs::Point p;
        	p.x = msg->pose[i].position.x;
        	p.y = msg->pose[i].position.y;
			p.z = msg->pose[i].position.z;

			points.points.push_back(p);

			marker_pub.publish(points);
		}
	}
	//ROS_ERROR("Kiiir %f", msg->pose[0].position.x);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "gazebo_ground_truth");
	ros::NodeHandle n;

	n.param("robot_name", robot_name, std::string("Not Available"));
	n.param("frame_id", frame_id, std::string("no_frame_id"));
	n.param("namespace_", namespace_, std::string("no_namespace"));

	sub = n.subscribe("/gazebo/model_states", 1000, chatterCallback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	ros::spin();
	return 0;
}