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

#define QUE_LEN 50

using namespace std;

float loadcell = 0;
float h_start = 0, h_step = 0, h_end = 0;
float height = 0;
int pwm1, pwm2, param_rate;
string testName;
bool UseLASER = false;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GE_Teststand");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher  pub_zd    = nh.advertise<std_msgs::Float64>("/zd", 5);
  ros::Publisher  pub_cmd   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  ros::Publisher  pub_ready = nh.advertise<std_msgs::Bool>("reading_ready", 5);
  ros::Subscriber sub_loadcell = nh.subscribe("/loadcell", 1, loadcell_callback);
  ros::Subscriber sub_height   = nh.subscribe("/height", 1, height_callback);

  n.getParam("startHeight", h_start);
  n.getParam("stepHeight", h_step);
  n.getParam("stopHeight", h_end);
  n.getParam("rate", param_rate);
  n.getParam("PWM1", pwm1);
  n.getParam("PWM2", pwm2);
  n.getParam("testName", testName);
  n.getParam("laserheight", UseLASER);

  ofstream fileHandle;
  string filePath = "/home/urc_admin/Documents/darc/XH/data/";
  filePath.append(testName);filePath.append(".txt");
  fileHandle.open(filePath.c_str());

  ros::Rate loop_rate(param_rate);

  cout<<endl<<"Writing into file: "<<filePath<<endl<<endl;

  tf::TransformListener tfListener;
  if(!UseLASER)
    tfListener.waitForTransform("vicon/baseboard/baseboard", "vicon/upsidedown/upsidedown", ros::Time(0), ros::Duration(10.0));
  int count = 0;
  float zd = h_start, init_loadcell, final_loadcell, dt;
  bool UP = true, StartTest = true, ReadingReady = false, MeasureDone = false;
  int PWM = pwm1;
  ros::Time startTime, currentTime;
  startTime = ros::Time::now();
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
      if(fabs(height-zd) > 0.002 && StartTest == true && dt<45){
	// Do nothing until reaching desired height
	std_msgs::Float64 msg_zd;
	msg_zd.data = zd+board_z;
	pub_zd.publish(msg_zd);
	geometry_msgs::Twist msg_cmd;
	msg_cmd.linear.z = 0;
	pub_cmd.publish(msg_cmd);
      }else{
	// Mark the initial value when test started
	if(StartTest == true){
	  // Wait until the loadcell is ready
	  if(ReadingReady == true){
	    init_loadcell = loadcell_read();
	    StartTest = false;
	    v_loadcell.clear();
	    // Mark start time
	    startTime = ros::Time::now();
	    geometry_msgs::Twist msg_cmd;
	    msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	    ROS_INFO("Motor start with PWM %d", PWM);
	  }else{
	    ROS_INFO_STREAM_THROTTLE(1,"Waiting 0 PWM loadcell reading.");
	    geometry_msgs::Twist msg_cmd;
	    msg_cmd.linear.z = 0;
	    pub_cmd.publish(msg_cmd);
	  }
	}else{
	  //cout<<"Current dt "<<dt<<endl;
	  //cout<<"Reading Ready is "<<ReadingReady<<endl;
	  if((dt > 3 && ReadingReady) || dt > 8){
	    MeasureDone = true;
	    final_loadcell = loadcell_read();
	    geometry_msgs::Twist msg_cmd;
	    msg_cmd.linear.z = 0;
	    pub_cmd.publish(msg_cmd);
	    // Write the height and loadcell value
	    fileHandle<<zd<<", "<<final_loadcell - init_loadcell<<endl;
	    ROS_INFO("Done.");
	  }else{
	    ROS_INFO_STREAM_THROTTLE(1,"Waiting loadcell reading.");
	    geometry_msgs::Twist msg_cmd;
	    msg_cmd.linear.z = PWM;
	    pub_cmd.publish(msg_cmd);
	  }
	}

	if(MeasureDone == true){
	  if(UP && zd < h_end-h_step/2.0){
	    zd += h_step;
	  }else if(fabs(zd - h_end)<h_step/2.0 && UP){
	    UP = false;
	    sleep(2);
	  }else if(zd > h_start+h_step/2.0){
	    zd -= h_step;
	  }else if(PWM == pwm1){// Only has two PWM inputs
	    UP = true;
	    PWM = pwm2;
	    sleep(2);
	  }else
	    ros::shutdown();
	  StartTest = true;
	  MeasureDone = false;
	  v_loadcell.clear();
	}

      }
      
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  fileHandle.close();
  return 0;
}
