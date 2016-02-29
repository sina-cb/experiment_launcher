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

    x = 0;
    y = 0;

    set_move_base_max_vel(0.4);

    //1
    geometry_msgs::Pose *waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 26.43;
    waypoint->position.y = 12.89;
    quat.setRPY(0, 0, -M_PI/3);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //2
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 29.91;
    waypoint->position.y = 9.78;
    quat.setRPY(0, 0, -M_PI/3);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //3
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 34.88;
    waypoint->position.y = 8.93;
    quat.setRPY(0, 0, -M_PI/2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //4
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 34.32;
    waypoint->position.y = 3.04;
    quat.setRPY(0, 0, (-2 * M_PI / 3));
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //5
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 28.74;
    waypoint->position.y = 4.90;
    quat.setRPY(0, 0, -M_PI);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //6
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 21.70;
    waypoint->position.y = 6.97;
    quat.setRPY(0, 0, M_PI / 2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //7
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 22.72;
    waypoint->position.y = 12.97;
    quat.setRPY(0, 0, M_PI / 2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    // Loop Completed

    //8
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 29.95;
    waypoint->position.y = 12.93;
    quat.setRPY(0, 0, 0);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //9
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 34.66;
    waypoint->position.y = 12.92;
    quat.setRPY(0, 0, -M_PI/3);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //10
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 35.0825;
    waypoint->position.y = 6.25;
    quat.setRPY(0, 0, -M_PI/2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //11
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 34.32;
    waypoint->position.y = 3.04;
    quat.setRPY(0, 0, (-2 * M_PI / 3));
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //12
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 28.74;
    waypoint->position.y = 4.90;
    quat.setRPY(0, 0, -M_PI);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //13
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 21.70;
    waypoint->position.y = 6.97;
    quat.setRPY(0, 0, M_PI / 2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

    //14
    waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 23.06;
    waypoint->position.y = 12.81;
    quat.setRPY(0, 0, M_PI / 2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);
    velocities.push_back(0.4);

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

            quat.setX(waypoints[counter].orientation.x);
            quat.setY(waypoints[counter].orientation.y);
            quat.setZ(waypoints[counter].orientation.z);
            quat.setW(waypoints[counter].orientation.w);

            loaded_waypoint = true;
            publish_goal_flag = true;
            seeking_waypoint = true;
        }
    }

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

        set_move_base_max_vel(velocities[counter]);

        //        if (counter % 2 == 0){
        //            ROS_INFO("Reduce Speed!!!");
        //            set_move_base_max_vel(0.2);
        //        }else{
        //            ROS_INFO("Increase Speed!!!");
        //            set_move_base_max_vel(0.4);
        //        }

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
