#ifndef Floor5_ROBOT2_H
#define Floor5_ROBOT2_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <cmvision/Blobs.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class floor5_robot2
{
public:
    floor5_robot2();

    ros::Subscriber robot2_goal_status_sub, amcl_pose_sub;
    ros::Publisher goal_pub2_;


    void robot2_goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr &status_array2);
    void amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);

    std::string fixed_frame;
    double theta;
    double x, y;

    double my_pose_x, my_pose_y, my_pose_theta;


    tf::Quaternion quat;

    geometry_msgs::Pose waypoint1, waypoint2, waypoint3, waypoint4, robot2_final_goal;

    int robot2_goalStatus;
    std::string robot2_status_goal_id;

    bool seeking_waypoint_1, seeking_waypoint_2, seeking_waypoint_3, seeking_waypoint_4, robot2_seeking_final_goal, loaded_waypoint_1, loaded_waypoint_2, loaded_waypoint_3, loaded_waypoint_4, robot2_loaded_final_goal, publish_goal_flag;



};

#endif // Floor5_ROBOT2_H
