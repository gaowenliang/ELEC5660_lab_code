#include "trajectory.h"
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

ros::Publisher pub_position_cmd;
ros::Publisher pub_des_path;

ros::Time time_now, time_last, time_start;
Eigen::Vector3d pos_now, pos_start;
Eigen::Vector3d vel_now;
int traj_id;
bool is_init = false;
bool is_traj = false;

void
trajectory_draw( ros::Time now_t )
{
    double max_time_sec = 30.0;
    double dt           = 0.1;
    int pose_num        = max_time_sec / dt;

    nav_msgs::Path path_des;
    path_des.header.stamp    = now_t;
    path_des.header.frame_id = "world";

    for ( int seq_index = 0; seq_index < pose_num; ++seq_index )
    {

        Eigen::Vector3d desired_pos, desired_vel, desired_acc, _pos_start( 0.0, 0.0, 0.0 ), _vel_now( 0.0, 0.0, 0.0 );
        double time_end;
        trajectory_control( seq_index * dt, _pos_start, _vel_now, time_end, desired_pos, desired_vel, desired_acc );

        geometry_msgs::PoseStamped pose;
        pose.header.stamp       = now_t - ros::Duration( max_time_sec - seq_index * dt );
        pose.header.seq         = seq_index;
        pose.header.frame_id    = "world";
        pose.pose.position.x    = desired_pos.x( );
        pose.pose.position.y    = desired_pos.y( );
        pose.pose.position.z    = desired_pos.z( );
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        path_des.poses.push_back( pose );
    }
    pub_des_path.publish( path_des );
}

void
trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg )
{
    if ( is_init )
    {
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        time_start = time_now;
        pos_start  = pos_now;
        traj_id    = trigger_msg->header.seq + 1;
        is_traj    = true;
        trajectory_draw( time_now );
    }
}

void
odom_callback( const nav_msgs::Odometry::ConstPtr& odom_msg )
{
    time_now     = odom_msg->header.stamp;
    pos_now.x( ) = odom_msg->pose.pose.position.x;
    pos_now.y( ) = odom_msg->pose.pose.position.y;
    pos_now.z( ) = odom_msg->pose.pose.position.z;

    if ( is_init )
    {
        if ( is_traj )
        {
            ros::Duration delta_t = time_now - time_start;

            Eigen::Vector3d desired_pos, desired_vel, desired_acc;
            double time_end;
            bool traj_ok
            = trajectory_control( delta_t.toSec( ), pos_start, vel_now, time_end, desired_pos, desired_vel, desired_acc );

            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = time_now;
            position_cmd.header.frame_id = "world";
            if ( traj_ok )
            {
                position_cmd.position.x      = desired_pos.x( );
                position_cmd.position.y      = desired_pos.y( );
                position_cmd.position.z      = desired_pos.z( );
                position_cmd.velocity.x      = desired_vel.x( );
                position_cmd.velocity.y      = desired_vel.y( );
                position_cmd.velocity.z      = desired_vel.z( );
                position_cmd.acceleration.x  = desired_acc.x( );
                position_cmd.acceleration.y  = desired_acc.y( );
                position_cmd.acceleration.z  = desired_acc.z( );
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
                position_cmd.trajectory_id   = traj_id;
            }
            else
            {
                position_cmd.position.x      = pos_now.x( );
                position_cmd.position.y      = pos_now.y( );
                position_cmd.position.z      = pos_now.z( );
                position_cmd.velocity.x      = 0.0;
                position_cmd.velocity.y      = 0.0;
                position_cmd.velocity.z      = 0.0;
                position_cmd.acceleration.x  = 0.0;
                position_cmd.acceleration.y  = 0.0;
                position_cmd.acceleration.z  = 0.0;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
                position_cmd.trajectory_id   = traj_id;
            }
            pub_position_cmd.publish( position_cmd );
        }
    }
    else
    {
        is_init = true;
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "trajectory" );
    ros::NodeHandle n( "~" );

    ros::Subscriber sub_trigger = n.subscribe( "/traj_start_trigger", 100, trigger_callback );
    ros::Subscriber sub_odom    = n.subscribe( "/pos_vel_mocap/odom_TA", 100, odom_callback );

    pub_des_path     = n.advertise< nav_msgs::Path >( "/des_path", 100 );
    pub_position_cmd = n.advertise< quadrotor_msgs::PositionCommand >( "/position_cmd", 10 );

    ros::spin( );
    return 0;
}
