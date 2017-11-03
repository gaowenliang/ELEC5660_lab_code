#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub_odom;
ros::Publisher pub_path;

bool init_ok = false;

Eigen::Vector3d init_P, last_P;
Eigen::Quaterniond init_Q, last_Q;
ros::Time now_t, last_odom_t, last_path_t;
Eigen::Vector3d Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;

nav_msgs::Path run_path;

void
pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
    if ( !init_ok )
    {
        init_ok     = true;
        init_Q.w( ) = msg->pose.orientation.w;
        init_Q.x( ) = msg->pose.orientation.x;
        init_Q.y( ) = msg->pose.orientation.y;
        init_Q.z( ) = msg->pose.orientation.z;
        init_P.x( ) = msg->pose.position.x;
        init_P.y( ) = msg->pose.position.y;
        init_P.z( ) = msg->pose.position.z;
        last_P      = init_P;
        last_Q      = init_Q;
        last_odom_t = msg->header.stamp;
    }
    else
    {
        now_t = msg->header.stamp;

        Eigen::Vector3d now_P, P_w;
        Eigen::Quaterniond now_Q, Q_w;
        now_P.x( ) = msg->pose.position.x;
        now_P.y( ) = msg->pose.position.y;
        now_P.z( ) = msg->pose.position.z;
        now_Q.w( ) = msg->pose.orientation.w;
        now_Q.x( ) = msg->pose.orientation.x;
        now_Q.y( ) = msg->pose.orientation.y;
        now_Q.z( ) = msg->pose.orientation.z;
        // Q_w = init_Q.normalized().toRotationMatrix().transpose() * now_Q.normalized().toRotationMatrix();
        Q_w = now_Q.normalized( ).toRotationMatrix( );
        P_w = now_P - init_P;

        Eigen::Vector3d now_vel;
        now_vel = ( P_w - last_P ) / ( now_t - last_odom_t ).toSec( );
        //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
        //        std::cout << " now_vel " << now_vel << std::endl;

        /** velocity filter **/
        Vi0 = now_vel;
        Vo0 = ( Vi0 + Vi1 + Vi2 + Vi3 + Vi4 ) * 0.2;
        Vi4 = Vi3;
        Vi3 = Vi2;
        Vi2 = Vi1;
        Vi1 = Vi0;

        /*********************/
        nav_msgs::Odometry odom;
        odom.header.stamp            = now_t;
        odom.header.frame_id         = "world";
        odom.pose.pose.position.x    = P_w.x( );
        odom.pose.pose.position.y    = P_w.y( );
        odom.pose.pose.position.z    = P_w.z( );
        odom.pose.pose.orientation.w = Q_w.w( );
        odom.pose.pose.orientation.x = Q_w.x( );
        odom.pose.pose.orientation.y = Q_w.y( );
        odom.pose.pose.orientation.z = Q_w.z( );
        odom.twist.twist.linear.x    = Vo0.x( ); // now_vel.x();
        odom.twist.twist.linear.y    = Vo0.y( ); // now_vel.y();
        odom.twist.twist.linear.z    = Vo0.z( ); // now_vel.z();
        pub_odom.publish( odom );

        last_odom_t = now_t;
        last_P      = P_w;
        last_Q      = Q_w;

        ros::Duration delta_t = now_t - last_path_t;
        if ( delta_t.toSec( ) > 0.1 )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp       = now_t;
            pose.header.frame_id    = "world";
            pose.pose.orientation.x = odom.pose.pose.orientation.x;
            pose.pose.orientation.y = odom.pose.pose.orientation.y;
            pose.pose.orientation.z = odom.pose.pose.orientation.z;
            pose.pose.orientation.w = odom.pose.pose.orientation.w;
            pose.pose.position.x    = odom.pose.pose.position.x;
            pose.pose.position.y    = odom.pose.pose.position.y;
            pose.pose.position.z    = odom.pose.pose.position.z;

            run_path.header.stamp    = now_t;
            run_path.header.frame_id = "world";
            run_path.poses.push_back( pose );
            pub_path.publish( run_path );

            last_path_t = now_t;
        }
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "pos_vel_mocap" );
    ros::NodeHandle n( "~" );

    ros::Subscriber s1 = n.subscribe( "/Robot_1/pose", 100, pose_callback );

    pub_odom = n.advertise< nav_msgs::Odometry >( "odom_TA", 100 );
    pub_path = n.advertise< nav_msgs::Path >( "/mocap_path", 10 );

    ros::spin( );
    return 0;
}
