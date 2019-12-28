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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <numeric>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <crazyflie_driver/UpdateParams.h>

#define QUE_LEN 50
#define RADIUS 0.1016

using namespace std;
using namespace Eigen;

std_msgs::Float32MultiArray partial_GE_msg;
Vector3d quad_pose, obstacle_pose, quad_vel;
Eigen::Quaternion<double> quad_q(1,0,0,0);
bool obstacle_available = false, quad_available = false, Enable_PartialGE = false;
double Ca = 0.4, Cb = 2.2;

void quad_callback(const geometry_msgs::TransformStamped& message){
  if(!quad_available)
    quad_available = true;
  quad_pose << message.transform.translation.x, message.transform.translation.y, message.transform.translation.z;
  Eigen::Quaternion<double> q(message.transform.rotation.w, message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z);
  quad_q = q;
}

void quad_vel_callback(const geometry_msgs::TwistStamped& message){
  quad_vel << message.twist.linear.x, message.twist.linear.y, message.twist.linear.z;
}

void obstacle_callback(const geometry_msgs::TransformStamped& message){
  if(!obstacle_available)
    obstacle_available = true;
  obstacle_pose << message.transform.translation.x, message.transform.translation.y, message.transform.translation.z;
}

float get_Partial_GE(Vector3d rotor, Vector3d obstacle, double obstacle_x, double obstacle_y){
  float partial = 0;
  // Calculate the partial GE here
  double partial_GE = 1.0;
  double z = 0;
  // GE model: Ca1 = -3/(2*theta_0)*(Ca0+1)*mu1+Ca0;
  // Assuming constant rotor tip velocity of 30 m/s
  double theta_0 = 23.0/180.0*M_PI;
  double v_mag = sqrt(quad_vel(0)*quad_vel(0)+quad_vel(1)*quad_vel(1));
  double Ca_mu = -3.0/(2.0*theta_0)*(Ca+1)*v_mag/30.0 + Ca;
  // First check in GE or not
  if(rotor(0) <= obstacle(0) + obstacle_x/2.0 - RADIUS && rotor(0) >= obstacle(0) - obstacle_x/2.0 + RADIUS && \
     rotor(1) <= obstacle(1) + obstacle_y/2.0 - RADIUS && rotor(1) >= obstacle(1) - obstacle_y/2.0 + RADIUS){
    // If fully over obstacle
    z = rotor(2) - obstacle(2);
    z = z < 0 ? 0 : z;
  }else if((rotor(0) >= obstacle(0) + obstacle_x/2.0 + RADIUS || rotor(0) <= obstacle(0) - obstacle_x/2.0 - RADIUS) || \
	   (rotor(1) >= obstacle(1) + obstacle_y/2.0 + RADIUS || rotor(1) <= obstacle(1) - obstacle_y/2.0 - RADIUS)){
    // If far from obstacle
    z = rotor(2);
    z = z < 0 ? 0 : z;
  }else{
    z = rotor(2) - obstacle(2);
    z = z < 0 ? 0 : z;
    // In partial ground effect
    if(rotor(0) <= obstacle(0) + obstacle_x/2.0 - RADIUS && rotor(0) >= obstacle(0) - obstacle_x/2.0 + RADIUS){
      // On left or right edge
      double d = 2.0*RADIUS - fabs(obstacle_y/2.0 + RADIUS - fabs(obstacle(1)-rotor(1)));
      partial_GE = (RADIUS*RADIUS*acos((d-RADIUS)/RADIUS)-(d-RADIUS)*sqrt(RADIUS*RADIUS-(d-RADIUS)*(d-RADIUS))) / (M_PI*RADIUS*RADIUS);
    }else if(rotor(1) <= obstacle(1) + obstacle_y/2.0 - RADIUS && rotor(1) >= obstacle(1) - obstacle_y/2.0 + RADIUS){
      // On front or rear edge
      double d = 2.0*RADIUS - fabs(obstacle_x/2.0 + RADIUS - fabs(obstacle(0)-rotor(0)));
      partial_GE = (RADIUS*RADIUS*acos((d-RADIUS)/RADIUS)-(d-RADIUS)*sqrt(RADIUS*RADIUS-(d-RADIUS)*(d-RADIUS))) / (M_PI*RADIUS*RADIUS);
    }else{
      // On corner
      double dx = fabs(obstacle_x/2.0 + RADIUS - fabs(obstacle(0)-rotor(0)));
      double dy = fabs(obstacle_y/2.0 + RADIUS - fabs(obstacle(1)-rotor(1)));
      partial_GE = dx*dy / (M_PI*RADIUS*RADIUS);
      partial_GE = partial_GE > 1 ? 1 : partial_GE;
    }
  }
  partial = partial_GE*Ca_mu*exp(-Cb*z/RADIUS)+1;
  return sqrt(partial);
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

void param_update(ros::NodeHandle n, ros::ServiceClient serv_param){
  n.setParam("/crazyflie/partialGE/partialEnable", Enable_PartialGE);
  Server_Update(serv_param, string("partialGE/partialEnable"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "partial_GE");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher  pub_partial_GE = nh.advertise<std_msgs::Float32MultiArray>("partial_GE", 5);
  ros::Subscriber sub_quad       = nh.subscribe("/vicon/Jackquad/Jackquad", 1, quad_callback);
  ros::Subscriber sub_quad_vel   = nh.subscribe("velocity", 1, quad_vel_callback);
  ros::Subscriber sub_obstacle   = nh.subscribe("/vicon/Block/Block", 1, obstacle_callback);
  ros::ServiceClient serv_param  = nh.serviceClient<crazyflie_driver::UpdateParams>("/crazyflie/update_params");
  
  double obstacle_x = 1.0, obstacle_y = 1.0;
  n.getParam("obstacle_length_x", obstacle_x);
  n.getParam("obstacle_length_y", obstacle_y);
  n.getParam("IGE_Ca", Ca);
  n.getParam("IGE_Cb", Cb);
  n.getParam("partialEnable", Enable_PartialGE);

  ros::Rate loop_rate(50);

  int count = 0;
  Eigen::MatrixXd rotor_body(3,4);
  rotor_body << 0.13,   -0.13,  -0.13,   0.13,
               -0.105, -0.105,  0.105,  0.105,
                0.01,    0.01,   0.01,   0.01;
  ros::Time startTime, currentTime, waitTime;
  startTime = ros::Time::now();
  waitTime = ros::Time::now();
  param_update(n, serv_param);
  while (ros::ok())
    {
      // make sure all transform are available
      if(obstacle_available && quad_available && Enable_PartialGE){
	// get rotor pose
	Eigen::Matrix3d R = quad_q.toRotationMatrix();
	Eigen::MatrixXd rotor = R * rotor_body;
	Eigen::MatrixXd body_translate(3,4);
	body_translate << quad_pose, quad_pose, quad_pose, quad_pose;
	rotor = rotor + body_translate;
	// calculate partial GE
	for(int i = 0; i < 4; i++){
	  partial_GE_msg.data.push_back(get_Partial_GE(rotor.col(i), obstacle_pose, obstacle_x, obstacle_y));
	  
	  //cout<<rotor.col(i).transpose()<<endl;
	}
	// publish partial GE
	pub_partial_GE.publish(partial_GE_msg);
	partial_GE_msg.data.clear();
      }
      if(count%50 == 0)
	param_update(n, serv_param);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}
