#include "floor5_robot2.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define DELTA_LINEAR 0.5
#define DELTA_ANGULAR 0.52 //30 degrees

floor5_robot2::floor5_robot2()
{
    ros::NodeHandle n;

    robot2_goal_status_sub = n.subscribe("/robot2/move_base/status", 1, &floor5_robot2::robot2_goal_status_Callback, this);
    amcl_pose_sub = n.subscribe("/robot2/amcl_pose", 1, &floor5_robot2::amcl_Callback, this);

    goal_pub2_ = n.advertise<geometry_msgs::PoseStamped>("/robot2/move_base_simple/goal", 1);

    fixed_frame = std::string("/map");

    theta = -M_PI/2;
    x=0;
    y=0;

    geometry_msgs::Pose *waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 1.0;
    waypoints.push_back(*waypoint);

    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 2.0;
    waypoints.push_back(*waypoint);

    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 3.0;
    waypoints.push_back(*waypoint);

    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 4.0;
    waypoints.push_back(*waypoint);

    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 5.0;
    waypoints.push_back(*waypoint);

    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 0.60;
    waypoint->position.y = 6.0;
    waypoints.push_back(*waypoint);

//    geometry_msgs::Pose *waypoint = new geometry_msgs::Pose();
//    waypoint->position.x = 0.61;
//    waypoint->position.y = 0.01;
//    waypoints.push_back(*waypoint);

//    waypoint = new geometry_msgs::Pose();
//    waypoint->position.x = 0.60;
//    waypoint->position.y = 10.93;
//    waypoints.push_back(*waypoint);

//    waypoint = new geometry_msgs::Pose();
//    waypoint->position.x = 1.13;
//    waypoint->position.y = 13.32;
//    waypoints.push_back(*waypoint);

//    waypoint = new geometry_msgs::Pose();
//    waypoint->position.x = 2.13;
//    waypoint->position.y = 13.32;
//    waypoints.push_back(*waypoint);

//    waypoint = new geometry_msgs::Pose();
//    waypoint->position.x = 5.13;
//    waypoint->position.y = 13.32;
//    waypoints.push_back(*waypoint);

    counter = 0;

    seeking_waypoint = false;
    loaded_waypoint  = false;

    my_pose_x = 0;
    my_pose_y = 0;
    my_pose_theta = 0;
}


void floor5_robot2::robot2_goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr &status_array2){
    actionlib_msgs::GoalStatus status_list_;
    geometry_msgs::PoseStamped goal;

    if(!status_array2->status_list.empty()){
        status_list_ = status_array2->status_list.back();


        if(robot2_goalStatus != status_list_.status
                || robot2_status_goal_id.compare(status_list_.goal_id.id) != 0 ){
            ROS_INFO("Robot2 Status: %d \tGoal ID: %s", status_list_.status, status_list_.goal_id.id.c_str());
        }

        robot2_goalStatus = status_list_.status;
        robot2_status_goal_id = status_list_.goal_id.id;
    }
    else{
        ROS_INFO("Robot2 No status received ... publishing goal again");
        publish_goal_flag = true;
    }

    if(!seeking_waypoint){
        if(!loaded_waypoint){
            x = waypoints[counter].position.x;
            y = waypoints[counter].position.y;
            theta = -M_PI;

            loaded_waypoint = true;
            publish_goal_flag = true;
            seeking_waypoint = true;
        }
    }

    quat.setRPY(0.0, 0.0, theta);
    tf::Stamped<tf::Pose> p1 = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
    tf::poseStampedTFToMsg(p1, goal);

    if(publish_goal_flag){
        goal_pub2_.publish(goal);
        ROS_INFO("Robot 2 seeking point : %f, %f ", x, y);
        publish_goal_flag = false;
    }

    if(robot2_goalStatus == 1){
        publish_goal_flag = false;
    }

    if (seeking_waypoint
            && my_pose_x <= (waypoints[counter].position.x + DELTA_LINEAR)
            && my_pose_x >= (waypoints[counter].position.x - DELTA_LINEAR)
            && my_pose_y <= (waypoints[counter].position.y + DELTA_LINEAR)
            && my_pose_y >= (waypoints[counter].position.y - DELTA_LINEAR)
            ){
        ROS_INFO("Robot2 reached Waypoint %d", (int)(counter));
        seeking_waypoint = false;
        loaded_waypoint  = false;

        if (counter % 2 == 0){
            ROS_INFO("Reduce Speed!!!");
            set_move_base_max_vel(0.2);
        }else{
            ROS_INFO("Increase Speed!!!");
            set_move_base_max_vel(0.4);
        }

        counter++;
        if (counter >= waypoints.size()){
            ROS_INFO("Reached the final goal!");
            ROS_INFO("This process is done!");
            exit(0);
        }
    }

}

void floor5_robot2::set_move_base_max_vel(double new_vel){

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "max_trans_vel";
    double_param.value = new_vel;
    conf.doubles.push_back(double_param);

    double_param.name = "max_vel_x";
    double_param.value = new_vel;
    conf.doubles.push_back(double_param);

//    double_param.name = "kurtana_roll_joint";
//    double_param.value = yaw;
//    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/robot2/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

}

void floor5_robot2::amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose){

    geometry_msgs::PoseWithCovarianceStamped amcl_pose_ = *amcl_pose;

    my_pose_x = amcl_pose_.pose.pose.position.x;
    my_pose_y = amcl_pose_.pose.pose.position.y;
    my_pose_theta = amcl_pose_.pose.pose.orientation.z;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "floor5_robot2");

    floor5_robot2 robot2_runner;


    ros::spin();

    return 0;
}
