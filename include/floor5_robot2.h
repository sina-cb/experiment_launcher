#ifndef Floor5_ROBOT2_H
#define Floor5_ROBOT2_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/String.h>
#include <cmvision/Blobs.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
using namespace std;

class floor5_robot2
{
public:
    floor5_robot2();

    ros::Subscriber robot2_goal_status_sub, amcl_pose_sub;
    ros::Publisher goal_pub2_;
    ros::Publisher robot_1_communication;

    void robot2_goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr &status_array2);
    void amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    void set_move_base_max_vel(double new_vel);

    std::string fixed_frame;
    double theta;
    double x, y;

    double my_pose_x, my_pose_y, my_pose_theta;

    tf::Quaternion quat;

    vector<geometry_msgs::Pose> waypoints;
    vector<double> velocities;
    size_t counter;

    int robot2_goalStatus;
    std::string robot2_status_goal_id;

    bool seeking_waypoint, loaded_waypoint, publish_goal_flag;
};

#endif // Floor5_ROBOT2_H
