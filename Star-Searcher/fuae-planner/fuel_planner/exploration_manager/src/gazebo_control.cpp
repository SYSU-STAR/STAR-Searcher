
// Author:Zixuan Zhuang
// Date:2023/4/8
//参考RACER中的simulator_light，可实现gazebo中的完美跟随效果
// move_base
// goal触发之前，机体朝向由我们手动给出，此处是yaw旋转-90度；触发之后由算法接管
#include "quadrotor_msgs/PositionCommand.h"
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
using namespace std;

ros::Subscriber _cmd_sub, _cllick_sub;
ros::Publisher _odom_pub;
ros::Publisher _game_flag_pub;
ros::ServiceClient client;

quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y, _init_z;
double _init_roll = 0.0, _init_pitch = 0.0, _init_yaw = 0.0;
bool rcv_cmd = false;
bool rcv_trigger = false;
tf::Quaternion quat =
    tf::createQuaternionFromRPY(_init_roll, _init_pitch, _init_yaw);

void clickSub(const geometry_msgs::PoseStamped &msg) {
  rcv_trigger = true;
  std_msgs::Bool start_flag;
  start_flag.data = true;
  _game_flag_pub.publish(start_flag);
}
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd) {
  rcv_cmd = true;
  _cmd = cmd;
}

void pubOdom() //通过调用gazebo相关接口的方式发布无人机odom
{
  gazebo_msgs::ModelState pose;
  pose.model_name = "ardrone";
  pose.reference_frame = "world";
  if (rcv_cmd && rcv_trigger) {
    pose.pose.position.x = _cmd.position.x;
    pose.pose.position.y = _cmd.position.y;
    pose.pose.position.z = _cmd.position.z;
    Eigen::Vector3d alpha =
        Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y,
                        _cmd.acceleration.z) +
        9.8 * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
    Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
    Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    Eigen::Vector3d zB = xB.cross(yB); //机体坐标系下z轴的单位向量
    Eigen::Matrix3d R;
    R.col(0) = xB;
    R.col(1) = yB;
    R.col(2) = zB;
    Eigen::Quaterniond q(R);
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

    pose.twist.linear.x = _cmd.velocity.x;
    pose.twist.linear.y = _cmd.velocity.y;
    pose.twist.linear.z = _cmd.velocity.z;

    Eigen::Vector3d ang = _cmd.yaw_dot * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d tmp = ang.cross(zB);
    double ang_x = ((-1) * tmp).dot(yB);
    double ang_y = tmp.dot(xB);
    double ang_z = (_cmd.yaw_dot * Eigen::Vector3d(0, 0, 1)).dot(zB);

    pose.twist.angular.x = ang_x;
    pose.twist.angular.y = ang_y;
    pose.twist.angular.z = ang_z;
  } else {
    std::cout << "NOT RECEIVE Trigger" << std::endl;
    pose.pose.position.x = _init_x;
    pose.pose.position.y = _init_y;
    pose.pose.position.z = _cmd.position.z;

    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();

    pose.twist.linear.x = 0.0;
    pose.twist.linear.y = 0.0;
    pose.twist.linear.z = 0.0;

    pose.twist.angular.x = 0.0;
    pose.twist.angular.y = 0.0;
    pose.twist.angular.z = 0.0;
  }
  _odom_pub.publish(pose);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_control_uav_state");
  ros::NodeHandle n("~");

  n.param("init_x", _init_x, 0.0);
  n.param("init_y", _init_y, 0.0);
  n.param("init_z", _init_z, 0.0);

  _cmd_sub = n.subscribe("command", 1, rcvPosCmdCallBack);
  _cllick_sub = n.subscribe("/move_base_simple/goal", 10, clickSub);
  _odom_pub =
      n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
  _game_flag_pub = n.advertise<std_msgs::Bool>("/start_flag", 10);
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    pubOdom();
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}