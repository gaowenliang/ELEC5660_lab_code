#ifndef     POSE__H_
#define     POSE__H_

#include "Eigen/Eigen"

Eigen::Vector3d     Qbw2RPY( Eigen::Quaterniond orientation )
{
    double  q0, q1, q2, q3;
    q0      = orientation.w();
    q1      = orientation.x();
    q2      = orientation.y();
    q3      = orientation.z(); 
    Eigen::Vector3d     rpy;
    rpy.x()     =   atan2( 2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2) );
    rpy.y()     =   asin( 2*(q0*q2 - q3*q1) );
    rpy.z()     =   atan2( 2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3) );
    return  rpy;
}

Eigen::Quaterniond  RPY2Qbw( Eigen::Vector3d rpy )
{
    Eigen::Matrix3d     RX, RY, RZ;
    RX      = Eigen::AngleAxisd( rpy.x(), Eigen::Vector3d::UnitX() );
    RY      = Eigen::AngleAxisd( rpy.y(), Eigen::Vector3d::UnitY() );
    RZ      = Eigen::AngleAxisd( rpy.z(), Eigen::Vector3d::UnitZ() );
    return (Eigen::Quaterniond)(RX*RY*RZ);
}
#endif


