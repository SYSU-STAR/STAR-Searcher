#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
using namespace std;

ros::Subscriber _cmd_sub, _cllick_sub;
ros::Publisher  _odom_pub;

quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y, _init_z;
void pubOdom()    
{
     gazebo_msgs::ModelState pose;
     pose.model_name  = "ardrone";
     pose.reference_frame = "world";
     pose.pose.position.x = 2;
     pose.pose.position.y = 2;
     pose.pose.position.z = 1;

     pose.pose.orientation.w = 1;
     pose.pose.orientation.x = 0;
     pose.pose.orientation.y = 0;
     pose.pose.orientation.z = 0;

     pose.twist.linear.x = 0.0;
     pose.twist.linear.y = 0.0;
     pose.twist.linear.z = 0.0;

     pose.twist.angular.x = 0.0;
     pose.twist.angular.y = 0.0;
     pose.twist.angular.z = 0.0;
    
     _odom_pub.publish(pose);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_control_uav_state");
    ros::NodeHandle n( "~" );

    n.param("init_x", _init_x,  0.0);
    n.param("init_y", _init_y,  0.0);
    n.param("init_z", _init_z,  0.0);

    _odom_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;

    
}