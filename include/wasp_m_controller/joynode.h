#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>

geometry_msgs::Twist command_velocity;
ros::Publisher  teleop_cmd, teleop_cmd2;
double start_button;
double joy_check=0;
double previous_start_button=0;
double waypointstart_button;
int flag = 1;
int teleop_state =0;
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void TeleopVelocity(int flag);
