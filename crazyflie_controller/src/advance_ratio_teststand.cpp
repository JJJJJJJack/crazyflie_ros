#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <numeric>
#include "lowpass.h"
#include <crazyflie_driver/Battery.h>
#include <crazyflie_driver/UpdateParams.h>
#include "LPfilter.h"

#define QUE_LEN 50

using namespace std;

float loadcell = 0, pitch = 0;
float h_start = 0, h_step = 0, h_end = 0;
float height = 0, speed = 0, vd = 0, last_v = 0;
float omega = 0;
float power = 0, raw_power = 0;
int pwm1, pwm2, param_rate = 50;
string testName;
bool UseLASER = false;
bool LogOmega = false;
bool SpeedChecked = false;
vector<float> v_loadcell;
geometry_msgs::PoseStamped current_pos, old_pos;
ros::Time old_time;
//LPfilter speed_filter(1.0,0.02);

void loadcell_callback(const std_msgs::Float64& msg)
{
  //cout<<"msg.data: "<<msg.data<<" loadcell: "<<loadcell<<endl;
  loadcell = msg.data;
  if(v_loadcell.size()<QUE_LEN)
    v_loadcell.push_back(loadcell);
  else{
    v_loadcell.erase(v_loadcell.begin());
    v_loadcell.push_back(loadcell);
    }
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

void omega_callback(const std_msgs::Float64& msg)
{
  omega = msg.data;
}

/*void position_callback(const geometry_msgs::PoseStamped &msg)
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
  }*/


void speed_callback(const std_msgs::Float64& msg)
{
    speed = msg.data;
}

bool loadcell_check(float tolerance)
{
  float v_max = 0, v_min = 0;
  //Loadcell vector should have size defined on top
  if(v_loadcell.size()<QUE_LEN)
    return false;
  v_max = *max_element(v_loadcell.begin(), v_loadcell.end());
  v_min = *min_element(v_loadcell.begin(), v_loadcell.end());
  if(fabs(v_max-v_min) < tolerance)
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

// This function is to ramp the desired linear velocity
float rampVd(float desired_speed, float update_rate){
  float output = 0;
  if(last_v <= desired_speed)
    output = last_v + 0.1/update_rate;
  else if(last_v > desired_speed)
    output = last_v - 0.1/update_rate;
  last_v = output;
  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "advance_ratio_teststand");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher  pub_zd      = nh.advertise<std_msgs::Float64>("/zd", 5);
  ros::Publisher  pub_cmd     = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  ros::Publisher  pub_ready   = nh.advertise<std_msgs::Bool>("reading_ready", 5);
  ros::Publisher  pub_vd      = nh.advertise<std_msgs::Float64>("/vd", 5);
  ros::Publisher  pub_control = nh.advertise<std_msgs::Bool>("/enable_control", 5);
  ros::Subscriber sub_loadcell  = nh.subscribe("/loadcell", 1, loadcell_callback);
  ros::Subscriber sub_height    = nh.subscribe("/height", 1, height_callback);
  ros::Subscriber sub_omega     = nh.subscribe("/rotor_freq", 1, omega_callback);
  ros::Subscriber sub_power     = nh.subscribe("/crazyflie/ExtBat", 1, power_callback);
  ros::Subscriber sub_speed     = nh.subscribe("/speed", 1, speed_callback);
  ros::ServiceClient serv_param = nh.serviceClient<crazyflie_driver::UpdateParams>("/crazyflie/update_params");
  //ros::Subscriber sub_position  = nh.subscribe("/vrpn_client_node/Jack_GE/pose", 1, position_callback);

  n.getParam("startHeight", h_start);
  n.getParam("stepHeight", h_step);
  n.getParam("stopHeight", h_end);
  n.getParam("rate", param_rate);
  n.getParam("PWM1", pwm1);
  n.getParam("PWM2", pwm2);
  n.getParam("testName", testName);
  n.getParam("laserheight", UseLASER);
  n.getParam("logging_omega", LogOmega);
  n.getParam("rotorPitch", pitch);
  n.getParam("desiredSpeed", vd);
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

  tf::TransformListener tfListener;
  if(!UseLASER)
    tfListener.waitForTransform("vicon/baseboard/baseboard", "vicon/upsidedown/upsidedown", ros::Time(0), ros::Duration(10.0));
  int count = 0;
  float zd = h_start, init_loadcell, final_loadcell, dt, wait_dt;
  bool UP = true, StartTest = true, ReadingReady = false, MeasureDone = false, Enable_GE = false, Enable_servo = false;
  int PWM = pwm1, ReadingCount = 0;
  geometry_msgs::Twist msg_cmd;
  std_msgs::Bool enable_control;
  ros::Time startTime, currentTime, waitTime;
  startTime = ros::Time::now();
  waitTime = ros::Time::now();
  //Initialize param update to enable GE test
  param_update(n, serv_param, 0, 0, 0);// z: 0, left: 0, right: 0;
  Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
  while (ros::ok())
    {
      //Define maximum tolerance in absolute gram
      float tolerance = 50.0;
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
      std_msgs::Float64 vd_msg;
      vd_msg.data = rampVd(vd, param_rate);
      pub_vd.publish(vd_msg);
      // If the beam does not passed the speed check, wait until the beam start following
      if(SpeedChecked == false){
	if(fabs(speed-vd) > 0.1){
	  ROS_INFO_STREAM_THROTTLE(5,"Waiting beam speed...");
	  enable_control.data = false;
	  pub_control.publish(enable_control);
	  ros::spinOnce();
	  loop_rate.sleep();
	  ++count;
	  continue;
	}else{
	  ROS_INFO("Beam ready..");
	  enable_control.data = true;
	  pub_control.publish(enable_control);
	  param_update(n, serv_param, 0, 0, 0);// z: 0, left: 0, right: 0;
	  Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
	  // Rest for 5 seconds to make sure the beam is finally in steady state
	  sleep(5);
	  SpeedChecked = true;
	}
      }

      if(fabs(height-zd) > 0.005 && StartTest == true && dt < 40.0){
	// Do nothing until reaching desired height
      	std_msgs::Float64 msg_zd;
      	msg_zd.data = zd+board_z;
      	pub_zd.publish(msg_zd);
      	msg_cmd.linear.z = 0;
      	pub_cmd.publish(msg_cmd);
	enable_control.data = true;
	pub_control.publish(enable_control);
	n.setParam("/crazyflie/GE_ANDO/GE_thrust", 0);
	Server_Update(serv_param, string("GE_ANDO/GE_thrust"));
	param_update(n, serv_param, 0, 0, 0);// z: 0, left: 0, right: 0;
	ROS_INFO_STREAM_THROTTLE(5,"Waiting height...");
      }else{
	// Disable the height control when measuring
	enable_control.data = false;
	pub_control.publish(enable_control);
	// Mark the initial value when test started
	if(StartTest == true){
	  // Wait until the loadcell is ready
	  if(ReadingReady == true){
	    if(ReadingCount >= 4){
	      init_loadcell = loadcell_read();
	      Enable_GE = Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
	      ROS_INFO("Initial loadcell reading: %f", init_loadcell);
	      StartTest = false;
	      v_loadcell.clear();
	      // Mark start time
	      startTime = ros::Time::now();
	      waitTime =  ros::Time::now();
	      ReadingCount = 0;
	      ROS_INFO("Motor start with PWM %d", PWM);
	    }else
	      ReadingCount++;
	  }else{
	    ROS_INFO_STREAM_THROTTLE(5,"Waiting 0 PWM loadcell reading.");
	    msg_cmd.linear.z = 0;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, 0, 0, 0);//left: ang, right: ang; z: 0;
	    if(Enable_GE == false || Enable_servo == false){
	      Enable_GE = Server_Update(serv_param, string("GE_ANDO/GETEST_ENABLE"));
	      Enable_servo = Server_Update(serv_param, string("servoSet/servoEnable"));
	    }
	  }
	}else{
	  //cout<<"Current dt "<<dt<<endl;
	  //cout<<"Reading Ready is "<<ReadingReady<<endl;
	  if(fabs(loadcell_read()-init_loadcell) < 50){
	    // reset start time if no loadcell increase detected.
	    startTime = ros::Time::now();
	    if(msg_cmd.linear.z < PWM)
	      msg_cmd.linear.z += 500;
	    else
	      msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, 0, 0, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
	  }
	  if((dt > 6 && ReadingReady) || dt > 10){
	    if(ReadingCount >= 4){
	      ReadingCount = 0;
	      MeasureDone = true;
	      final_loadcell = loadcell_read();
	      ROS_INFO("Final loadcell reading: %f", final_loadcell);
	      msg_cmd.linear.x = 0;
	      msg_cmd.linear.y = 0;
	      msg_cmd.linear.z = 0;
	      pub_cmd.publish(msg_cmd);
	      param_update(n, serv_param, 0, 0, 0);//left: 0, right: 0; z: 0;
	      // Write height, loadcell and omega value
	      if(LogOmega == true)
		fileHandle<<zd<<", "<<final_loadcell - init_loadcell<<", "<<omega<<endl;
	      else
		fileHandle<<zd<<", "<<final_loadcell - init_loadcell<<endl;
	      ROS_INFO("Done.");
	    }else
	      ReadingCount++;
	  }else{
	    ROS_INFO_STREAM_THROTTLE(5,"Waiting loadcell reading.");
	    if(msg_cmd.linear.z < PWM)
	      msg_cmd.linear.z += 500;
	    else
	      msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	    param_update(n, serv_param, 0, 0, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
	    if(wait_dt > 15){
	      // We wait long enough, should restart the process
	      msg_cmd.linear.z = 0;
	      pub_cmd.publish(msg_cmd);
	      param_update(n, serv_param, 0, 0, msg_cmd.linear.z);//left: ang, right: ang; z: msg_cmd.linear.z;
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
	    //sleep(4);
	  }else if(zd > h_start+h_step/2.0){
	    zd -= h_step;
	  }else if(PWM == pwm1){// Only has two PWM inputs
	    UP = true;
	    PWM = pwm2;
	    //sleep(4);
	  }else{
	    vd = 0;
	    if(vd_msg.data <= 0.1)
	      ros::shutdown();
	  }
	  StartTest = true;
	  MeasureDone = false;
	  v_loadcell.clear();
	  // Re-enable the height control when measuring
	  enable_control.data = true;
	  pub_control.publish(enable_control);
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
