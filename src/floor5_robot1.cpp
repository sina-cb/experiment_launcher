#include "floor5_robot1.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

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

#define MIN_DISTANCE 1.0
#define MAX_DISTANCE 10.0
#define MAX_SPEED 0.6
#define SPEED_OFFSET 0.4

#define FINAL_STAGE_KICKOFF_X 0
#define FINAL_STAGE_KICKOFF_Y 6

#define DELTA_LINEAR 0.5
#define DELTA_ANGULAR 0.52 //30 degrees

namespace floor5_robot1{


Floor5_Robot1::Floor5_Robot1()
{
    ROS_ERROR("11111");
    // Subscribers
    laser_sub = n.subscribe("/scan", 1, &Floor5_Robot1::laser_Callback,this);
    color_blob_sub = n.subscribe("/blobs", 1, &Floor5_Robot1::color_blob_Callback, this);

    // Publishers
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    robot1_exp_state_pub_ = n.advertise<std_msgs::String>("/robot1_experiment_state", 1);

    fixed_frame = std::string("/map");

    follow_or_not = true;

    x       = 0;
    y       = 0;

    geometry_msgs::Pose *waypoint = new geometry_msgs::Pose();
    waypoint->position.x = 5.35;
    waypoint->position.y = 1.10;
    quat.setRPY(0, 0, -M_PI/2);
    waypoint->orientation.x = quat.getX();
    waypoint->orientation.y = quat.getY();
    waypoint->orientation.z = quat.getZ();
    waypoint->orientation.w = quat.getW();
    waypoints.push_back(*waypoint);

    counter = 0;

    publish_goal_flag = true;

    seeking_waypoint = false;
    loaded_waypoint  = false;
    following_stage = true;

    color_centroid_x = -99;
    color_centroid_y = -99;
    leader_distance = 0.0;
    leader_theta = 1.0;
    last_seen_theta = 0.0;
    color_seen_time = 0.0;
    nested_amcl_publish_timestamp = 0.0;

    my_pose_x = 0.0;
    my_pose_y = 0.0;
}

void Floor5_Robot1::communication_2_Callback(const std_msgs::String &communication){


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

}

void Floor5_Robot1::set_move_base_max_vel(double new_vel){

}

void Floor5_Robot1::laser_Callback(const sensor_msgs::LaserScanConstPtr& laser_scan){

    ROS_ERROR("AAAA");

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
    }

    if(color_centroid_x >= 0.0){

        for(int beam_no = 0 ; beam_no < max_beams ; beam_no++){

            current_range = laser_scan->ranges.at(beam_no);

            if(min_measured_range > current_range
                    && beam_no > std::floor(max_beams/5)
                    && beam_no < std::ceil(4*max_beams/5)
                    ){
                min_measured_range = current_range;
            }

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

    if(counter > 0
            && leader_distance > 0
            && leader_distance <= laser_scan->range_max
            && follow_or_not){
        seeking_waypoint = false;
        following_stage = true;

        ROS_INFO("Robot1 switching to follower behaviour now because we found the leader again!");
    }

    if(following_stage){

        int max_lost_time = 1;
        if( (current_timestamp - color_seen_time) > max_lost_time){ // not seen leader for more than 60 seconds
            ROS_ERROR("----Lost sight of the leader for %d second, we're using the last known nested_particle_pose and go towards it----", max_lost_time);

            //            leader_distance = std::sqrt(std::pow(leader_pose_x - my_pose_x, 2) + std::pow(leader_pose_y - my_pose_y, 2));
            //            double temp_theta = std::asin((leader_pose_y - my_pose_y) / leader_distance);
            //            last_seen_theta = -1 * (my_pose_theta - temp_theta);

            double angle_correction = std::asin((leader_pose_x - my_pose_x) / std::sqrt(std::pow(leader_pose_x - my_pose_x, 2)
                                                                                        + std::pow(leader_pose_y - my_pose_y, 2)));

            //            geometry_msgs::Pose *waypoint = new geometry_msgs::Pose();
            //            waypoint->position.x = leader_pose_x;
            //            waypoint->position.y = leader_pose_y;
            //            quat.setRPY(0, 0, (leader_theta + angle_correction));
            //            waypoint->orientation.w = quat.getW();
            //            waypoint->orientation.x = quat.getX();
            //            waypoint->orientation.y = quat.getY();
            //            waypoint->orientation.z = quat.getZ();
            //            waypoints.push_back(*waypoint);

            seeking_waypoint = false;
            loaded_waypoint  = false;
            following_stage = true;

            set_move_base_max_vel(0.3);

            std::stringstream state_stream;
            std_msgs::String state_msg;
            state_stream << "Lost sight of the leader, we're using the last known nested_particle_pose and go towards it!";
            state_msg.data = state_stream.str();
            robot1_exp_state_pub_.publish(state_msg);
        }

        else{
            bool use_move_base = false;

            if(min_measured_range <= MIN_DISTANCE
                    || leader_distance <= MIN_DISTANCE){
                if(leader_distance > 0){
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;

                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = 2 * last_seen_theta;
                }

                else{
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.linear.y = 0.0;
                    cmd_vel_.linear.z = 0.0;

                    cmd_vel_.angular.x = 0.0;
                    cmd_vel_.angular.y = 0.0;
                    cmd_vel_.angular.z = 2 * last_seen_theta;
                }
            }

            else{
                cmd_vel_.linear.x = leader_distance * (MAX_SPEED / MAX_DISTANCE) + SPEED_OFFSET;
                cmd_vel_.linear.y = 0.0;
                cmd_vel_.linear.z = 0.0;


                cmd_vel_.angular.x = 0.0;
                cmd_vel_.angular.y = 0.0;
                cmd_vel_.angular.z = last_seen_theta * std::max(1 / cmd_vel_.linear.x , 1.0);
            }

            if(!use_move_base){
                if (!std::isnan(leader_distance)){
                    vel_pub_.publish(cmd_vel_);
                }else{
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.angular.z = last_seen_theta * std::max(1 / cmd_vel_.linear.x , 1.0);
                }
            }

            ROS_INFO("distance, speed: %f | %f ", leader_distance, cmd_vel_.linear.x);
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

    std::stringstream state_stream;
    std_msgs::String state_msg;

    if(seeking_waypoint){
        state_stream << "seeking waypoint " << counter;
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }
    else if(following_stage){
        state_stream << "following stage";
        state_msg.data = state_stream.str();
        robot1_exp_state_pub_.publish(state_msg);
    }
}

void Floor5_Robot1::nested_amcl_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &nested_amcl_pose){

    geometry_msgs::PoseWithCovarianceStamped nested_amcl_pose_ = *nested_amcl_pose;


    leader_pose_x = nested_amcl_pose_.pose.pose.position.x;
    leader_pose_y = nested_amcl_pose_.pose.pose.position.y;
    leader_pose_theta = nested_amcl_pose_.pose.pose.orientation.z;
}

} // End of Namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "floor5_robot1");

    floor5_robot1::Floor5_Robot1 robot1_runner;

    ros::spin();

    return 0;
}
