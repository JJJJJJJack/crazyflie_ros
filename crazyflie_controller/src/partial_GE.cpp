/*
  File: Calculating partial ground effect for each rotor
  Note: All calculations are defined in NWU frame
  Author: Jack He
  Date created: 12/27/2019
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <numeric>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#define QUE_LEN 50

using namespace std;
using namespace Eigen;

std_msgs::Float32MultiArray partial_GE;
Vector3d quad_pose, obstacle_pose;
Eigen::Quaternion<double> quad_q(1,0,0,0);
bool obstacle_available = false, quad_available = false;

void quad_callback(const geometry_msgs::TransformStamped& message){
  if(!quad_available)
    quad_available = true;
  quad_pose << message.transform.translation.x, message.transform.translation.y, message.transform.translation.z;
  Eigen::Quaternion<double> q(message.transform.rotation.w, message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z);
  quad_q = q;
}

void obstacle_callback(const geometry_msgs::TransformStamped& message){
  if(!obstacle_available)
    obstacle_available = true;
  obstacle_pose << message.transform.translation.x, message.transform.translation.y, message.transform.translation.z;
}

float get_Partial_GE(Vector3d rotor, Vector3d obstacle){
  float partial = 0;
  // Calculate the partial GE here
  return partial;
}
/*
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
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "partial_GE");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher  pub_partial_GE = nh.advertise<std_msgs::Float32MultiArray>("partial_GE", 5);
  ros::Subscriber sub_quad       = nh.subscribe("/vicon/Jackquad/Jackquad", 1, quad_callback);
  ros::Subscriber sub_obstacle   = nh.subscribe("/vicon/hand/hand", 1, obstacle_callback);
  //ros::ServiceClient serv_param  = nh.serviceClient<crazyflie_driver::UpdateParams>("/crazyflie/update_params");
  
  //n.setParam("/crazyflie/GE_ANDO/GETEST_ENABLE", true);

  ros::Rate loop_rate(50);

  int count = 0;
  geometry_msgs::Twist msg_cmd;
  Eigen::MatrixXd rotor_body(3,4);
  rotor_body << 0.13,   -0.13,  -0.13,   0.13,
               -0.105, -0.105,  0.105,  0.105,
                0,          0,      0,      0;
  ros::Time startTime, currentTime, waitTime;
  startTime = ros::Time::now();
  waitTime = ros::Time::now();
  while (ros::ok())
    {
      // make sure all transform are available
      if(obstacle_available && quad_available){
	// get rotor pose
	Eigen::Matrix3d R = quad_q.toRotationMatrix();
	Eigen::MatrixXd rotor = R * rotor_body;
	Eigen::MatrixXd body_translate(3,4);
	body_translate << quad_pose, quad_pose, quad_pose, quad_pose;
	rotor = rotor + body_translate;
	// calculate partial GE
	for(int i = 0; i < 4; i++){
	  partial_GE.data.push_back(get_Partial_GE(rotor.col(i), obstacle_pose));
	  cout<<rotor.col(i).transpose()<<endl;
	}
	cout<<endl;
	// publish partial GE
	pub_partial_GE.publish(partial_GE);
	partial_GE.data.clear();
      }
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}
