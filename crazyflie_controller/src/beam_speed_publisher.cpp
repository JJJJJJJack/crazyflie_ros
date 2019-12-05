#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <cmath>
#include <numeric>
#include "LPfilter.h"

using namespace std;

float speed = 0;

geometry_msgs::PoseStamped current_pos, old_pos;
ros::Time old_time;
LPfilter speed_filter(1.0,0.02);

void position_callback(const geometry_msgs::PoseStamped &msg)
{
  current_pos = msg;
  ros::Time current_time = ros::Time::now();
  current_pos = msg;
  float dis = sqrt(pow(current_pos.pose.position.x-old_pos.pose.position.x,2)+
		   pow(current_pos.pose.position.y-old_pos.pose.position.y,2)+
		   0*pow(current_pos.pose.position.z-old_pos.pose.position.z,2));
  float dt = current_time.toSec()-old_time.toSec();
  // Limit the maximum speed to 5 m/s
  if(dis < 0.1)
    speed = speed_filter.update(dis/dt);
  old_pos = current_pos;
  old_time = current_time;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "beam_speed_publisher");

  ros::NodeHandle nh;
  ros::Publisher  pub_speed = nh.advertise<std_msgs::Float64>("/speed", 5);
  ros::Subscriber sub_position  = nh.subscribe("/vrpn_client_node/Jack_GE/pose", 1, position_callback);

  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
    {
      //Define maximum tolerance in percentage
      std_msgs::Float64 speed_msg;
      speed_msg.data = speed;
      pub_speed.publish(speed_msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}
