#ifndef Floor5_ROBOT1_H
#define Floor5_ROBOT1_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>

#include <cmvision/Blobs.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <vector>
using namespace std;

namespace floor5_robot1{

class Floor5_Robot1
{
public:
    Floor5_Robot1();

    ros::NodeHandle n;

    int max_beams;
    double straight_prob;
    double right_prob;
    double left_prob;

    int current_goal;
    actionlib_msgs::GoalID current_goal_id;

    std::string status_goal_id, robot1_status_goal_id;
    int robot1_goalStatus;

    bool seeking_waypoint, loaded_waypoint, publish_goal_flag;

    bool following_stage;
    bool follow_or_not;

    ros::Publisher vel_pub_;
    ros::Publisher goal_pub1_;
    ros::Publisher cancel_goal_pub_;
    ros::Publisher robot1_exp_state_pub_;

    ros::Subscriber laser_sub;
    ros::Subscriber goal_status_sub;
    ros::Subscriber color_blob_sub;
    ros::Subscriber amcl_sub;
    ros::Subscriber nested_amcl_sub;
    ros::Subscriber robot3_communication;
    ros::Subscriber robot1_communication;

    std::string fixed_frame;
    double theta;
    double x, y;

    double my_pose_x, my_pose_y, my_pose_theta;

    double leader_pose_x, leader_pose_y, leader_pose_theta;

    tf::Quaternion quat, quat2;

    geometry_msgs::Twist cmd_vel_;
    vector<geometry_msgs::Pose> waypoints;
    size_t counter;

    int color_centroid_x, color_centroid_y;
    double leader_distance, leader_theta , last_seen_theta;
    long int color_seen_time, nested_amcl_publish_timestamp;

    void laser_Callback(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& status_array);
    void color_blob_Callback(const cmvision::BlobsConstPtr& Blobs);
    void amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    void nested_amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &nested_amcl_pose);
    void communication_3_Callback(const std_msgs::String &communication);
    void communication_2_Callback(const std_msgs::String &communication);

    void robot2_goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& status_array2);
    void set_move_base_max_vel(double new_vel);

};

}


#endif // Floor5_ROBOT1_H
