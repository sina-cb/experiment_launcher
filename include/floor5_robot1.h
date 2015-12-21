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


namespace floor5_robot1{

class Floor5_Robot1
{
public:
    Floor5_Robot1();

    int max_beams;
    double straight_prob;
    double right_prob;
    double left_prob;
    bool publish_goal_flag, set_next_goal;

    int robot1_goalStatus, robot2_goalStatus;
    int current_goal;

    actionlib_msgs::GoalID current_goal_id;

    std::string
    status_goal_id,
    robot2_status_goal_id;

    bool loaded_waypoint_1, reached_waypoint_1, reached_current_goal, loaded_final_goal, robot2_loaded_final_goal;
    bool load_next_goal;

    bool seeking_waypoint_1;
    int attempting_waypoint1;

    bool following_stage;
    bool seeking_final_goal, robot2_seeking_final_goal;

    ros::Publisher vel_pub_;
    ros::Publisher goal_pub1_;

    ros::Publisher cancel_goal_pub_;

    ros::Publisher robot1_exp_state_pub_;

    ros::Subscriber laser_sub;
    ros::Subscriber goal_status_sub;
    ros::Subscriber robot2_goal_status_sub;

    ros::Subscriber color_blob_sub;

    ros::Subscriber amcl_sub;

    geometry_msgs::Twist cmd_vel_;


    geometry_msgs::Pose waypoint1, final_goal;

    std::string fixed_frame;
    double theta;
    double x, y;
    tf::Quaternion quat, quat2;



    int color_centroid_x, color_centroid_y;
    double leader_distance, leader_theta , last_seen_theta;

    long int color_seen_time, nested_amcl_publish_timestamp;

    double forward_vel, angular_vel;


    double my_pose_x, my_pose_y, my_pose_theta;


    void laser_Callback(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& status_array);
    void color_blob_Callback(const cmvision::BlobsConstPtr& Blobs);
    void amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);

    void robot2_goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& status_array2);
};

}


#endif // Floor5_ROBOT1_H
