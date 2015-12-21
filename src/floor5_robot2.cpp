#include "floor5_robot2.h"


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


    waypoint1.position.x = 3.0;
    waypoint1.position.y = 17.3;

    waypoint2.position.x = -4.0;
    waypoint2.position.y = 17.3;

    waypoint3.position.x = -5.8;
    waypoint3.position.y = 15;

    waypoint4.position.x = -5.8;
    waypoint4.position.y = 7;

    robot2_final_goal.position.x = -7;
    robot2_final_goal.position.y = 5;

    seeking_waypoint_1 = true;
    seeking_waypoint_2 = false;
    seeking_waypoint_3 = false;
    seeking_waypoint_4 = false;

    loaded_waypoint_1 =false;
    loaded_waypoint_2 =false;
    loaded_waypoint_3 =false;
    loaded_waypoint_4 =false;


    robot2_seeking_final_goal = false;
    robot2_loaded_final_goal = false;

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
            ROS_INFO("*******Robot2 Status: %d \tGoal ID: %s", status_list_.status, status_list_.goal_id.id.c_str());
        }

        robot2_goalStatus = status_list_.status;
        robot2_status_goal_id = status_list_.goal_id.id;
    }
    else{
        ROS_INFO("*******Robot2 No status received...publishing goal again");
        publish_goal_flag = true;
    }


    if(seeking_waypoint_1){
        if(!loaded_waypoint_1){
            x = waypoint1.position.x;
            y = waypoint1.position.y;
            theta = -M_PI;

            loaded_waypoint_1 = true;
            publish_goal_flag = true;
        }
    }

    //        else

    if(seeking_waypoint_2){
        if(!loaded_waypoint_2){
            x = waypoint2.position.x;
            y = waypoint2.position.y;
            theta = -M_PI;

            loaded_waypoint_2 = true;
            publish_goal_flag = true;
        }
    }


    if(seeking_waypoint_3){
        if(!loaded_waypoint_3){
            x = waypoint3.position.x;
            y = waypoint3.position.y;
            theta = -M_PI/2;

            loaded_waypoint_3 = true;
            publish_goal_flag = true;
        }
    }

    if(seeking_waypoint_4){
        if(!loaded_waypoint_4){
            x = waypoint4.position.x;
            y = waypoint4.position.y;
            theta = -M_PI/2;

            loaded_waypoint_4 = true;
            publish_goal_flag = true;
        }
    }

    if(robot2_seeking_final_goal){
        if(!robot2_loaded_final_goal){
            x = robot2_final_goal.position.x;
            y = robot2_final_goal.position.y;
            theta = -M_PI/2;

            robot2_loaded_final_goal = true;
            publish_goal_flag = true;
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
        //reached_current_goal = false;
    }


    if (seeking_waypoint_1
            && my_pose_x <= (waypoint1.position.x + DELTA_LINEAR)
            && my_pose_x >= (waypoint1.position.x - DELTA_LINEAR)
            && my_pose_y <= (waypoint1.position.y+DELTA_LINEAR)
            && my_pose_y >= (waypoint1.position.y-DELTA_LINEAR)
            //                  && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
            //                  && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)
            ){
        ROS_INFO("Robot2 reached Waypoint 1!!!-----");
        seeking_waypoint_2 = true;
        seeking_waypoint_1 = false;
        //                exit(0);
    }

    if (seeking_waypoint_2
            && my_pose_x <= (waypoint2.position.x + DELTA_LINEAR)
            && my_pose_x >= (waypoint2.position.x - DELTA_LINEAR)
            && my_pose_y <= (waypoint2.position.y+DELTA_LINEAR)
            && my_pose_y >= (waypoint2.position.y-DELTA_LINEAR)
            //                  && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
            //                  && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)
            ){
        ROS_INFO("Robot2 reached Waypoint 2!!!-----");
        seeking_waypoint_3 = true;
        seeking_waypoint_2 = false;
        //                exit(0);
    }


    if (seeking_waypoint_3
            && my_pose_x <= (waypoint3.position.x + DELTA_LINEAR)
            && my_pose_x >= (waypoint3.position.x - DELTA_LINEAR)
            && my_pose_y <= (waypoint3.position.y+DELTA_LINEAR)
            && my_pose_y >= (waypoint3.position.y-DELTA_LINEAR)
            //                  && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
            //                  && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)
            ){
        ROS_INFO("Robot2 reached Waypoint 3!!!-----");
        seeking_waypoint_4 = true;
        seeking_waypoint_3 = false;
        //                exit(0);
    }


    if (seeking_waypoint_4
            && my_pose_x <= (waypoint4.position.x + DELTA_LINEAR)
            && my_pose_x >= (waypoint4.position.x - DELTA_LINEAR)
            && my_pose_y <= (waypoint4.position.y+DELTA_LINEAR)
            && my_pose_y >= (waypoint4.position.y-DELTA_LINEAR)
            //                  && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
            //                  && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)
            ){
        ROS_INFO("Robot2 reached Waypoint 4!!!-----");
        robot2_seeking_final_goal = true;
        seeking_waypoint_4 = false;
        //                exit(0);
    }



    if(robot2_goalStatus == 3){
        if (robot2_seeking_final_goal
                //                    && my_pose_x <= (final_goal.position.x + DELTA_LINEAR)
                //                    && my_pose_x >= (final_goal.position.x - DELTA_LINEAR)
                //                    && my_pose_y <= (final_goal.position.y+DELTA_LINEAR)
                //                    && my_pose_y <= (final_goal.position.x-DELTA_LINEAR)
                //                    && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
                //                    && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)
                ){
            ROS_INFO_ONCE("Robot2 reached final Goal!!!-----");
            //                exit(0);
        }
    }



    if(robot2_goalStatus == 4
            || robot2_goalStatus == 5
            || robot2_goalStatus == 9){
        publish_goal_flag = true;
    }



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
