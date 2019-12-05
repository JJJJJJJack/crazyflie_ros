#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <numeric>
#include "lowpass.h"
#include <crazyflie_driver/Battery.h>
#include <crazyflie_driver/UpdateParams.h>

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_run");
  ros::NodeHandle nh;
  cout<<"Start running..."<<endl;
  //while(ros::ok()){
  //}
  sleep(20);
  return 0;
}
