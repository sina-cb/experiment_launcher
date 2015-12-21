#include "floor5_robot1.h"



#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
//This is = (57/640)*(M_PI/180)
#define RADIANS_PER_PIXEL 0.001554434

//KPM: Since Kinect camera has a field of view of 57 degrees
// This is = (-28.5 * M_PI)/180
//#define COLOR_MIN_ANGLE −0.497419
#define COLOR_MIN_ANGLE −0.497418837
// This is = (28.5 * M_PI)/180
#define COLOR_MAX_ANGLE  0.497418837

#define MIN_DISTANCE 0.5
#define MAX_DISTANCE 10.0
#define MAX_SPEED 1.0

#define FINAL_STAGE_KICKOFF_X 0
#define FINAL_STAGE_KICKOFF_Y 6

#define DELTA_LINEAR 0.5
#define DELTA_ANGULAR 0.52 //30 degrees

namespace floor5_robot1{


Floor5_Robot1::Floor5_Robot1()
{

    ros::NodeHandle n;

    //for comanding the base
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1);

    //ros::Subscriber action_sub = n.subscribe("cmd_vel", 100, cmd_vel_Callback);

    laser_sub = n.subscribe("/robot1/scan", 1, &Floor5_Robot1::laser_Callback,this);

    goal_status_sub = n.subscribe("/robot1/move_base/status", 1, &Floor5_Robot1::goal_status_Callback, this);




    color_blob_sub = n.subscribe("/robot1/blobs", 1, &Floor5_Robot1::color_blob_Callback, this);

    amcl_sub = n.subscribe("/robot1/amcl_pose", 1, &Floor5_Robot1::amcl_Callback, this);



    goal_pub1_ = n.advertise<geometry_msgs::PoseStamped>("/robot1/move_base_simple/goal", 1);

    cancel_goal_pub_ = n.advertise<actionlib_msgs::GoalID>("/robot1/move_base/cancel", 1);

    robot1_exp_state_pub_ = n.advertise<std_msgs::String>("/robot1_experiment_state", 1);



    fixed_frame = std::string("/map");

    theta = -M_PI/2;
    x=0;
    y=0;


    waypoint1.position.x = 5.5;
    waypoint1.position.y = 18;

    final_goal.position.x = -5;
    final_goal.position.y = 1;

    publish_goal_flag = true;

    seeking_waypoint_1 = true;
    following_stage = false;
    seeking_final_goal = false;

    attempting_waypoint1 = 1;

    load_next_goal = true;

    loaded_waypoint_1 = false;
    loaded_final_goal = false;
    reached_waypoint_1 = false;
    reached_current_goal = false;



    color_centroid_x = -99;
    color_centroid_y = -99;
    leader_distance = 0.0;
    leader_theta = 1.0;
    last_seen_theta = 0.0;
    color_seen_time = 0.0;
    nested_amcl_publish_timestamp = 0.0;

    forward_vel = 0.0;
    angular_vel = 0.0;

    my_pose_x = 0.0;
    my_pose_y = 0.0;


}


void Floor5_Robot1::goal_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr &status_array){

    std::stringstream state_stream;
    std_msgs::String state_msg;

    if(!following_stage){
        actionlib_msgs::GoalStatus status_list_;
        geometry_msgs::PoseStamped goal1;

        if(!status_array->status_list.empty()){
            status_list_ = status_array->status_list.back();

            if(robot1_goalStatus != status_list_.status
                    || current_goal_id.id.compare(status_list_.goal_id.id) != 0 ){
                ROS_INFO("*******Robot1 Status: %d \tGoal ID: %s", status_list_.status, status_list_.goal_id.id.c_str());
            }

            current_goal_id = status_list_.goal_id;
            robot1_goalStatus = status_list_.status;
            status_goal_id = status_list_.goal_id.id;

        }
        else{
            ROS_INFO("*******Robot1 No status received...publishing goal again");
            publish_goal_flag = true;
        }


        if(seeking_waypoint_1){
            if(!loaded_waypoint_1){
                x = waypoint1.position.x;
                y = waypoint1.position.y;
                theta = -M_PI/2;

                loaded_waypoint_1 = true;
                publish_goal_flag = true;
            }
        }

        else{
            if(seeking_final_goal){
                if(!loaded_final_goal){
                    x = final_goal.position.x;
                    y = final_goal.position.y;
                    theta = -M_PI/2;

                    loaded_final_goal = true;
                    publish_goal_flag = true;
                }
            }
        }



        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p1 = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        tf::poseStampedTFToMsg(p1, goal1);




        if(publish_goal_flag){
            goal_pub1_.publish(goal1);
            publish_goal_flag = false;
        }



        if(robot1_goalStatus == 1){
            publish_goal_flag = false;
            reached_current_goal = false;
        }

        if(robot1_goalStatus == 3){

            if(seeking_waypoint_1
                    && !following_stage
                    && !seeking_final_goal){
                if(my_pose_x <= (waypoint1.position.x + DELTA_LINEAR)
                        && my_pose_x >= (waypoint1.position.x - DELTA_LINEAR)
                        && my_pose_y <= (waypoint1.position.y+DELTA_LINEAR)
                        && my_pose_y >= (waypoint1.position.x-DELTA_LINEAR)
                        && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
                        && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)){

                    publish_goal_flag = true;
                    reached_current_goal = true;
                    reached_waypoint_1 = true;
                    seeking_waypoint_1 = false;
                    following_stage = true;

                }
            }

            if (seeking_final_goal
                    && my_pose_x <= (final_goal.position.x + DELTA_LINEAR)
                    && my_pose_x >= (final_goal.position.x - DELTA_LINEAR)
                    && my_pose_y <= (final_goal.position.y+DELTA_LINEAR)
                    && my_pose_y >= (final_goal.position.x-DELTA_LINEAR)
                    && my_pose_theta <= (goal1.pose.orientation.z + DELTA_ANGULAR)
                    && my_pose_theta >= (goal1.pose.orientation.z - DELTA_ANGULAR)){
                ROS_INFO("Robot1 reached final Goal!!! YAY!!!");

                state_stream << "reached_final_goal stopping";
                state_msg.data = state_stream.str();
                robot1_exp_state_pub_.publish(state_msg);

//                seeking_final_goal = false;

//                ros::shutdown();
            }
        }



        if(robot1_goalStatus == 5
                || robot1_goalStatus == 9){
            publish_goal_flag = true;
        }

        if(robot1_goalStatus == 4){
            if(seeking_waypoint_1){
                if(attempting_waypoint1 < 4){
                    attempting_waypoint1++;
                    publish_goal_flag = true;
                }
                else{
                    ROS_INFO("----Waypoint1 aborted too many times. Forcefully Shutting down as a crash.----");

                    state_stream << "waypoint1_aborted crashed";
                    state_msg.data = state_stream.str();
                    robot1_exp_state_pub_.publish(state_msg);

//                    ros::shutdown();
                }
            }
            else if(seeking_final_goal){
                ROS_INFO("---- Could not get a path plan to go to final goal. Shutting down.----");

                state_stream << "cannot_find_path_to_final_goal stopping";
                state_msg.data = state_stream.str();
                robot1_exp_state_pub_.publish(state_msg);

//                ros::shutdown();
            }
        }
    }

}




void Floor5_Robot1::color_blob_Callback(const cmvision::BlobsConstPtr &Blobs){
    cmvision::Blobs Blobs_ = *Blobs;
    cmvision::Blob temp_blob;

    int biggest_blob = 0;
    uint blob_area = 0;


    if(Blobs_.blob_count == 0){
        color_centroid_x = -99;
        color_centroid_y = -99;
        leader_theta = 1.0;
        leader_distance = 0.0;
    }

    else{

        color_centroid_x = 0.0;
        color_centroid_y = 0.0;

        for(uint i=0; i<Blobs_.blob_count; i++ ){
            temp_blob = Blobs_.blobs.at(i);

            //Contiguous landmark assumption
            color_centroid_x = color_centroid_x + temp_blob.x;
            color_centroid_y = color_centroid_y + temp_blob.y;

/*
            //Discontiguous landmark assumption
            if(temp_blob.area > blob_area){
                blob_area = temp_blob.area;
                color_centroid_x = temp_blob.x;
                color_centroid_y = temp_blob.y;
                biggest_blob = i;
            }
*/
        }

        color_centroid_x = color_centroid_x/Blobs_.blob_count;
        color_centroid_y = color_centroid_y/Blobs_.blob_count;

        leader_theta = ((IMAGE_WIDTH/2) - color_centroid_x) * RADIANS_PER_PIXEL;
        last_seen_theta = leader_theta;

        // Get last color seen timestamp
        struct timeval tp;
        gettimeofday(&tp, NULL);
        color_seen_time = tp.tv_sec; //get current timestamp in seconds

    }
    //ROS_INFO("colored blob centroid x,y: %d, %d \t leader_theta: %0.3f", color_centroid_x, color_centroid_y, leader_theta);




}



void Floor5_Robot1::laser_Callback(const sensor_msgs::LaserScanConstPtr& laser_scan){



    //bool recovery_flag = false;

    max_beams = laser_scan->ranges.size();

    double current_range = 0.0;
    double min_measured_range = 99.0;

    bool got_leader_distance = false;


    // Get current timestamp
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int current_timestamp = tp.tv_sec; //get current timestamp in seconds

    // no published data from nested_amcl for more than 120 seconds
    // ...nested_amcl must be hung/stuck/deadlocked
    if( (current_timestamp - nested_amcl_publish_timestamp) > 120){
        std::stringstream state_stream;
        std_msgs::String state_msg;

        ROS_INFO("----nested_amcl seems to be stuck. Forcefully Shutting down as a crash.----");

        state_stream << "nested_amcl_stuck reporting as crashed";
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }




    //    double beam_sum = 0;

    //ROS_INFO("%d", max_beams);

    if(color_centroid_x >= 0.0){

        for(int beam_no = 0 ; beam_no < max_beams ; beam_no++){

            current_range = laser_scan->ranges.at(beam_no);

            if(min_measured_range > current_range
                    && beam_no > std::floor(max_beams/5)
                    && beam_no < std::ceil(4*max_beams/5)
                    ){
                min_measured_range = current_range;
            }


            //        beam_sum += laser_scan->ranges[beam_no];
            //        if(laser_scan->ranges[beam_no] < (laser_scan->range_min + 0.5)){
            //            recovery_flag = true;
            //        }
            if( ( (laser_scan->angle_min + (beam_no * laser_scan->angle_increment)) >= leader_theta)
                    && !got_leader_distance){

                if(beam_no == 0){
                    beam_no++;
                }
                leader_distance = std::min(laser_scan->ranges.at(beam_no),
                                           laser_scan->ranges.at(beam_no-1));

                got_leader_distance = true;
            }

        }

    }

    if(seeking_waypoint_1
            && leader_distance>0
            && leader_distance <= laser_scan->range_max){
        seeking_waypoint_1 = false;
        following_stage = true;

        cancel_goal_pub_.publish(current_goal_id);

        ROS_INFO("Robot1 switching to follower behaviour now.");

    }


    if(!seeking_final_goal && following_stage){


        if( (current_timestamp - color_seen_time) > 60){ // not seen leader for more than 60 seconds

            std::stringstream state_stream;
            std_msgs::String state_msg;

            ROS_INFO("----Lost the leader for too long during following stage. Forcefully Shutting down as a crash.----");

            state_stream << "lost_leader_in_following_stage crashed";
            state_msg.data = state_stream.str();
            robot1_exp_state_pub_.publish(state_msg);
        }

        else{

            bool use_move_base = false;

            //ROS_INFO("leader distance, theta: %f \t %f",leader_distance, leader_theta);

            //        if(leader_distance > 0.0){

            //            following_stage = true;

            if(min_measured_range <= MIN_DISTANCE
                    || leader_distance <= MIN_DISTANCE){
                /*
                //                if(leader_distance < (MIN_DISTANCE + 1.0)){
                //                    cmd_vel_.linear.x = 0.0;
                //                    cmd_vel_.linear.y = 0.0;
                //                    cmd_vel_.linear.z = 0.0;
                //                }
                //                else{

*/
                if(leader_distance > 0){
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;

                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = 2*last_seen_theta;
                }

                else{
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;

                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = 2*last_seen_theta;
                }

                //                }
            }

            else{
                /*
                if(leader_distance < (MIN_DISTANCE + 2.0)){
                    cmd_vel_.linear.x = 0.6;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;
                }
                else{

                    cmd_vel_.linear.x = 1.0;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;

                }

*/

                cmd_vel_.linear.x = leader_distance*(MAX_SPEED/MAX_DISTANCE) + 0.55;
                cmd_vel_.linear.y = 0.0;
                cmd_vel_.linear.z = 0.0;


                cmd_vel_.angular.x = 0.0;
                cmd_vel_.angular.y = 0.0;
                cmd_vel_.angular.z = last_seen_theta* std::max(1/cmd_vel_.linear.x , 1.0);
            }

            /*

            if( color_centroid_x > IMAGE_WIDTH/3
                    && color_centroid_x < (2*IMAGE_WIDTH/3) ){

                cmd_vel_.angular.x = 0.0;
                cmd_vel_.angular.y = 0.0;
                cmd_vel_.angular.z = 0.0;
            }
            else{
                if(color_centroid_x <= IMAGE_WIDTH/3){
                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = 0.5;
                }
                else{
                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = -0.5;
                }
            }

            */


            //last_seen_theta = 0.0;

            if(!use_move_base){
                vel_pub_.publish(cmd_vel_);
            }
            /*
            else{
                theta = my_pose_theta + leader_theta;
                x = my_pose_x + cos(theta) * (leader_distance/2);
                y = my_pose_y + sin(theta) * (leader_distance/2);

                geometry_msgs::PoseStamped follower_goal;

                quat.setRPY(0.0, 0.0, theta);
                tf::Stamped<tf::Pose> p1 = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
                tf::poseStampedTFToMsg(p1, follower_goal);

                goal_pub1_.publish(follower_goal);

                use_move_base = false;

                //            ros::Rate loop_rate(0.2);

                //            loop_rate.sleep();
            }
            */

            ROS_DEBUG("distance, speed: %f | %f ", leader_distance, cmd_vel_.linear.x);

            //}

        }

    }

}



void Floor5_Robot1::amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose){

    geometry_msgs::PoseWithCovarianceStamped amcl_pose_ = *amcl_pose;

    // Get last nested_amcl_publish_timestamp
    struct timeval tp;
    gettimeofday(&tp, NULL);
    nested_amcl_publish_timestamp = tp.tv_sec; //get current timestamp in seconds

    my_pose_x = amcl_pose_.pose.pose.position.x;
    my_pose_y = amcl_pose_.pose.pose.position.y;
    my_pose_theta = amcl_pose_.pose.pose.orientation.z;

    //    ROS_INFO("amcl_pose (x, y, theta): (%f, %f, %f)", my_pose_x, my_pose_y, my_pose_theta);

    /*
    double angle = 0.0;

    if(following_stage && leader_distance > 0){
        angle = my_pose_theta + leader_theta;
        x = my_pose_x + cos(angle) * (leader_distance/2);
        y = my_pose_y + sin(angle) * (leader_distance/2);

        geometry_msgs::PoseStamped follower_goal;

        tf::Quaternion temp_quaternion;

        temp_quaternion.setRPY(0.0, 0.0, angle);
        tf::Stamped<tf::Pose> p1 = tf::Stamped<tf::Pose>(tf::Pose(temp_quaternion, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        tf::poseStampedTFToMsg(p1, follower_goal);

        goal_pub1_.publish(follower_goal);

    }

    */

    if(!seeking_final_goal
            && my_pose_x < -(FINAL_STAGE_KICKOFF_X)
            && my_pose_y < FINAL_STAGE_KICKOFF_Y){

        seeking_final_goal = true;
        following_stage = false;
        ROS_INFO("Robot1 seeking final goal.");
    }


    std::stringstream state_stream;
    std_msgs::String state_msg;

    if(seeking_waypoint_1){
        state_stream << "seeking_waypoint_1";
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }
    else if(following_stage){
        state_stream << "following_stage";
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }
    else if(seeking_final_goal){
        state_stream << "seeking_final_goal";
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }

}



}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "floor5_robot1");

    floor5_robot1::Floor5_Robot1 robot1_runner;


    ros::spin();

    return 0;
}
