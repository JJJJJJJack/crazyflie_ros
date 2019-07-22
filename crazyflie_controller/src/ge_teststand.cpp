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

#define QUE_LEN 50

using namespace std;

float loadcell = 0, pitch = 0;
float h_start = 0, h_step = 0, h_end = 0;
float height = 0;
float power = 0, raw_power = 0;
int pwm1, pwm2, param_rate;
string testName;
bool UseLASER = false;
bool LogPower = false;
vector<float> v_loadcell;

void loadcell_callback(const std_msgs::Float64& msg)
{
  loadcell = msg.data;
  if(v_loadcell.size()<QUE_LEN)
    v_loadcell.push_back(loadcell);
  else{
    v_loadcell.erase(v_loadcell.begin());
    v_loadcell.push_back(loadcell);
  }
  //cout<<v_loadcell.size()<<endl;
}

void height_callback(const std_msgs::Float64& msg)
{
  if(UseLASER)
    height = msg.data;
}

void power_callback(const crazyflie_driver::Battery& Bat)
{
  raw_power = Bat.power;
}

bool loadcell_check(float tolerance)
{
  float v_max = 0, v_min = 0;
  //Loadcell vector should have size defined on top
  if(v_loadcell.size()<QUE_LEN)
    return false;
  float average = accumulate(v_loadcell.begin(), v_loadcell.end(), 0.0)/v_loadcell.size(); 
  //Cannot divide 0
  if(average == 0)
    return false;
  v_max = *max_element(v_loadcell.begin(), v_loadcell.end());
  v_min = *min_element(v_loadcell.begin(), v_loadcell.end());
  if(fabs(v_max-v_min)/average < tolerance)
    return true;
  else
    return false;
}

float loadcell_read(void)
{
  return accumulate(v_loadcell.begin(), v_loadcell.end(), 0.0)/v_loadcell.size(); 
}

bool Server_Update(ros::ServiceClient serv_param, string param){
  //Setting up crazyflie to enable GE test
  crazyflie_driver::UpdateParams param_server;
  param_server.request.params.push_back(param);
  if(serv_param.call(param_server)){
    return param_server.response.result;
  }else{
    return false;
  }
}

void param_update(ros::NodeHandle n, ros::ServiceClient serv_param, float servo_left, float servo_right, int z){
  n.setParam("/crazyflie/GE_ANDO/GE_thrust", z);
  n.setParam("/crazyflie/servoSet/servoLeftAngle", servo_left);
  n.setParam("/crazyflie/servoSet/servoRightAngle", servo_right);
  n.setParam("/crazyflie/GE_ANDO/GETEST_ENABLE", true);
  Server_Update(serv_param, string("GE_ANDO/GE_thrust"));
  Server_Update(serv_param, string("servoSet/servoLeftAngle"));
  Server_Update(serv_param, string("servoSet/servoRightAngle"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ge_teststand");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher  pub_zd    = nh.advertise<std_msgs::Float64>("/zd", 5);
  ros::Publisher  pub_cmd   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  ros::Publisher  pub_ready = nh.advertise<std_msgs::Bool>("reading_ready", 5);
  ros::Subscriber sub_loadcell  = nh.subscribe("/loadcell", 1, loadcell_callback);
  ros::Subscriber sub_height    = nh.subscribe("/height", 1, height_callback);
  ros::Subscriber sub_power     = nh.subscribe("/crazyflie/ExtBat", 1, power_callback);
  ros::ServiceClient serv_param = nh.serviceClient<crazyflie_driver::UpdateParams>("/crazyflie/update_params");

  n.getParam("startHeight", h_start);
  n.getParam("stepHeight", h_step);
  n.getParam("stopHeight", h_end);
  n.getParam("rate", param_rate);
  n.getParam("PWM1", pwm1);
  n.getParam("PWM2", pwm2);
  n.getParam("testName", testName);
  n.getParam("laserheight", UseLASER);
  n.getParam("logging_power", LogPower);
  n.getParam("rotorPitch", pitch);
  n.setParam("/crazyflie/GE_ANDO/GETEST_ENABLE", true);
  n.setParam("/crazyflie/GE_ANDO/GE_thrust", 0);
  n.setParam("/crazyflie/servoSet/servoEnable", true);
  n.setParam("/crazyflie/servoSet/servoLeftAngle", 0.0f);
  n.setParam("/crazyflie/servoSet/servoRightAngle", 0.0f);

  ofstream fileHandle;
  string filePath = "/home/";
  filePath.append(getenv("USER"));
  filePath.append("/Documents/darc/XH/data/");
  filePath.append(testName);filePath.append(".txt");
  fileHandle.open(filePath.c_str());

  ros::Rate loop_rate(param_rate);

  LOWPASS LP_power(-20., 3e+2, 1, "power");
  cout<<endl<<"Writing into file: "<<filePath<<endl<<endl;
  float theta_left  = -0.6463*pitch-1.166;
  float theta_right = 0.8296*pitch-0.05022;
  cout<<"Sending desired pitch angle "<<theta_left<<" to left servo"<<endl<<endl;
  cout<<"Sending desired pitch angle "<<theta_right<<" to right servo"<<endl<<endl;

  tf::TransformListener tfListener;
  if(!UseLASER)
    tfListener.waitForTransform("vicon/baseboard/baseboard", "vicon/upsidedown/upsidedown", ros::Time(0), ros::Duration(10.0));
  int count = 0;
  float zd = h_start, init_loadcell, final_loadcell, dt, wait_dt;
  bool UP = true, StartTest = true, ReadingReady = false, MeasureDone = false, Enable_GE = false, Enable_servo = false;
  int PWM = pwm1;
  geometry_msgs::Twist msg_cmd;
  // Setting variable pitch for left using x(pitch) and right using y(roll)
  msg_cmd.linear.x = theta_left;
  msg_cmd.linear.y = theta_right;
  ros::Time startTime, currentTime, waitTime;
  startTime = ros::Time::now();
  waitTime = ros::Time::now();
  while (ros::ok())
    {
      //Define maximum tolerance in percentage
      float tolerance = 10.0/100.0;
      ReadingReady = loadcell_check(tolerance);
      std_msgs::Bool msg_ready;
      msg_ready.data = ReadingReady;
      pub_ready.publish(msg_ready);
      tf::StampedTransform transform, board;
      if(!UseLASER){
      	tfListener.lookupTransform("vicon/baseboard/baseboard", "vicon/upsidedown/upsidedown", ros::Time(0), transform);
      	tfListener.lookupTransform("/world", "vicon/baseboard/baseboard", ros::Time(0), board);
      	height = transform.getOrigin().z();
      }
      float board_z = board.getOrigin().z();
      currentTime = ros::Time::now();
      dt = currentTime.toSec() - startTime.toSec();
      wait_dt = currentTime.toSec() - waitTime.toSec();
      if(fabs(height-zd) > 0.002 && StartTest == true && dt<45){
	// Do nothing until reaching desired height
      	std_msgs::Float64 msg_zd;
      	msg_zd.data = zd+board_z;
      	pub_zd.publish(msg_zd);
      	msg_cmd.linear.z = 0;
      	pub_cmd.publish(msg_cmd);
	n.setParam("/crazyflie/GE_ANDO/GE_thrust", 0);
	Server_Update(serv_param, string("GE_ANDO/GE_thrust"));
	param_update(n, serv_param, 0, 0, 0);// z: 0, left: 0, right: 0;
      }else{
	// Mark the initial value when test started
	if(StartTest == true){
	  // Wait until the loadcell is ready
	  if(ReadingReady == true){
	    init_loadcell = loadcell_read();
	    Enable_GE = Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
	    ROS_INFO("Initial loadcell reading: %f", init_loadcell);
	    StartTest = false;
	    v_loadcell.clear();
	    // Mark start time
	    startTime = ros::Time::now();
	    waitTime =  ros::Time::now();
	    ROS_INFO("Motor start with PWM %d", PWM);
	  }else{
	    ROS_INFO_STREAM_THROTTLE(5,"Waiting 0 PWM loadcell reading.");
	    msg_cmd.linear.x = theta_left;
	    msg_cmd.linear.y = theta_right;
	    msg_cmd.linear.z = 0;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, theta_left, theta_right, 0);//left: ang, right: ang; z: 0;
	    if(Enable_GE == false || Enable_servo == false){
	      Enable_GE = Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
	      Enable_servo = Server_Update(serv_param, string("servoSet/servoEnable"));
	    }
	  }
	}else{
	  //cout<<"Current dt "<<dt<<endl;
	  //cout<<"Reading Ready is "<<ReadingReady<<endl;
	  if(fabs(loadcell_read()-init_loadcell) < 0.2){
	    // reset start time if no loadcell increase detected.
	    startTime = ros::Time::now();
	    msg_cmd.linear.x = theta_left;
	    msg_cmd.linear.y = theta_right;
	    if(msg_cmd.linear.z < PWM)
	      msg_cmd.linear.z += 500;
	    else
	      msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, theta_left, theta_right, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
	  }
	  if((dt > 4 && ReadingReady) || dt > 8){
	    MeasureDone = true;
	    final_loadcell = loadcell_read();
	    ROS_INFO("Final loadcell reading: %f", final_loadcell);
	    msg_cmd.linear.x = 0;
	    msg_cmd.linear.y = 0;
	    msg_cmd.linear.z = 0;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, 0, 0, 0);//left: 0, right: 0; z: 0;
	    // Write height, loadcell and battery power value
	    if(LogPower == true)
	      fileHandle<<zd<<", "<<final_loadcell - init_loadcell<<", "<<power<<endl;
	    else
	      fileHandle<<zd<<", "<<final_loadcell - init_loadcell<<endl;
	    ROS_INFO("Done.");
	  }else{
	    ROS_INFO_STREAM_THROTTLE(5,"Waiting loadcell reading.");
	    msg_cmd.linear.x = theta_left;
	    msg_cmd.linear.y = theta_right;
	    if(msg_cmd.linear.z < PWM)
	      msg_cmd.linear.z += 500;
	    else
	      msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, theta_left, theta_right, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
	    if(wait_dt > 15){
	      // We wait long enough, should restart the process
	      msg_cmd.linear.z = 0;
	      pub_cmd.publish(msg_cmd);
	      param_update(n, serv_param, theta_left, theta_right, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
	      ROS_WARN_STREAM_THROTTLE(1,"Restarting this PWM.");
	      waitTime = ros::Time::now();
	      sleep(10);
	    }
	  }
	}

	if(MeasureDone == true){
	  if(UP && zd < h_end-h_step/2.0){
	    zd += h_step;
	  }else if(fabs(zd - h_end)<h_step/2.0 && UP){
	    UP = false;
	    sleep(4);
	  }else if(zd > h_start+h_step/2.0){
	    zd -= h_step;
	  }else if(PWM == pwm1){// Only has two PWM inputs
	    UP = true;
	    PWM = pwm2;
	    sleep(4);
	  }else
	    ros::shutdown();
	  StartTest = true;
	  MeasureDone = false;
	  v_loadcell.clear();
	}

      }
      
      power = LP_power.filter(raw_power);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  fileHandle.close();
  return 0;
}
