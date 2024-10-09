#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>
#include <gazebo_msgs/SetModelState.h>
#include  <gazebo_msgs/ModelState.h>
#include <string>

using namespace std;

ros::ServiceClient client;

random_device rd;
default_random_engine eng(rd());
double x[5]={0.0,0.0,0.0,0.0,0.0}, y[5]={0.0,0.0,0.0,0.0,0.0};

int obs_num = 5;
int apriltag_xf_num = 5;
int apriltag_yf_num = 3;
uniform_real_distribution<double> distribution_x(0.0, 1.0);
uniform_real_distribution<double> distribution_y(0.0, 1.0);
vector<gazebo_msgs::SetModelState> box(obs_num);
vector<gazebo_msgs::SetModelState> apriltag_xf(apriltag_xf_num);
vector<gazebo_msgs::SetModelState> apriltag_yf(apriltag_yf_num);
gazebo_msgs::SetModelState obs_box;
vector<uniform_real_distribution<double>> _rand_x (obs_num,distribution_x);
vector<uniform_real_distribution<double>> _rand_y(obs_num,distribution_y);
vector<uniform_real_distribution<double>> _apriltag_rand_y(apriltag_xf_num,distribution_y);
vector<uniform_real_distribution<double>> _apriltag_rand_x(apriltag_yf_num,distribution_x);
vector<double> _x_l = {-3.0, 2.1, 2.1, -5.0, 7,7};
vector<double> _x_h = {1.5, 4.5, 7.2, -1.2, 9.5};
vector<double> _y_l = {0.0, -0.7, 2.8, 2.7, -5.5};
vector<double> _y_h = {1.8, 1.8, 5.5, 3.8, -3.7};
vector<double> _apriltag_xf_x = {5.05, -0.62, 5.52, -4.5, -9.5};
vector<double> _apriltag_xf_y_l = {1.0, 2.7, -3.8, -2.4, 3.5};
vector<double> _apriltag_xf_y_h = {2.0, 4.0, -2.5, -0.6, 4.7};
vector<double> _apriltag_yf_y = {-1.3, -3.05, -0.2};
vector<double> _apriltag_yf_x_l = {-4.0, -7.8, -10.0};
vector<double> _apriltag_yf_x_h = {-2.0, -4.8, -7.0};
void RandomMapGenerate()
{
   vector<string> obstacle_names = {"unit_box_0","unit_box_0_clone","unit_box_0_clone_0","unit_box_0_clone_2","unit_box_0_clone_3"};
   vector<string> apriltag_xf_names = {"static_apriltag_0","static_apriltag_1","static_apriltag_4","static_apriltag_2","static_apriltag_7"};
   vector<string> apriltag_yf_names = {"static_apriltag_3","static_apriltag_5","static_apriltag_6"};
   for(int i = 0; i < obs_num; i++)
   {
      _rand_x[i] = uniform_real_distribution<double>(_x_l[i],_x_h[i]);
      _rand_y[i] = uniform_real_distribution<double>(_y_l[i],_y_h[i]);
      x[i] = _rand_x[i](eng);
      y[i] = _rand_y[i](eng);
      box[i].request.model_state.model_name = obstacle_names[i];
      box[i].request.model_state.reference_frame = "world";
      box[i].request.model_state.pose.position.x = x[i];
      box[i].request.model_state.pose.position.y = y[i];
   }
   for(int i = 0; i < apriltag_xf_num; i++)
   {
      _apriltag_rand_y[i] = uniform_real_distribution<double>(_apriltag_xf_y_l[i],_apriltag_xf_y_h[i]);
      y[i] = _apriltag_rand_y[i](eng);
      apriltag_xf[i].request.model_state.model_name = apriltag_xf_names[i];
      apriltag_xf[i].request.model_state.reference_frame = "world";
      apriltag_xf[i].request.model_state.pose.position.x = _apriltag_xf_x[i];
      apriltag_xf[i].request.model_state.pose.position.y =  y[i];
      apriltag_xf[i].request.model_state.pose.position.z = 1.2;
   }
   for(int i = 0; i < apriltag_yf_num; i++)
   {
      _apriltag_rand_x[i] = uniform_real_distribution<double>(_apriltag_yf_x_l[i],_apriltag_yf_x_h[i]);
      x[i] = _apriltag_rand_x[i](eng);
      apriltag_yf[i].request.model_state.model_name = apriltag_yf_names[i];
      apriltag_yf[i].request.model_state.reference_frame = "world";
      apriltag_yf[i].request.model_state.pose.position.x = x[i];
      apriltag_yf[i].request.model_state.pose.position.y =  _apriltag_yf_y[i];
      apriltag_yf[i].request.model_state.pose.position.z = 1.2;
   }
   ros::Duration(3).sleep();
   for(int i = 0; i < apriltag_xf_num; i++)
   {
      client.call(apriltag_xf[i]);
      ros::Duration(0.2).sleep();
   }
   for(int i = 0; i < apriltag_yf_num; i++)
   {
      client.call(apriltag_yf[i]);
      ros::Duration(0.2).sleep();
   }
   for(int i = 0; i < obs_num; i++)
   {
      client.call(box[i]);
      ros::Duration(0.2).sleep();
   }
   obs_box.request.model_state.model_name = "unit_box";
   obs_box.request.model_state.reference_frame = "world";
   obs_box.request.model_state.pose.position.x = -10.03;
   obs_box.request.model_state.pose.position.y = apriltag_xf[4].request.model_state.pose.position.y;
   client.call(obs_box);
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_obstacle_generate");
   ros::NodeHandle n( "~" );
   client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
   RandomMapGenerate();
}

