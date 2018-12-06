#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <cmath>

#include "pid.hpp"
#include "lowpass.h"

//#define DIRECT_GE_COMPENSATE

using namespace std;

double get(
	   const ros::NodeHandle& n,
	   const std::string& name) {
  double value;
  n.getParam(name, value);
  return value;
}

struct JOY {
  float x;
  float y;
  float z;
  float yaw;
} Joy_input;

class Controller
{
public:

  Controller(
	     const std::string& worldFrame,
	     const std::string& frame,
	     const ros::NodeHandle& n)
    : m_worldFrame(worldFrame)
    , m_frame(frame)
    , m_pubNav()
    , m_pubPose()
    , m_pubPendulum()
    , m_listener()
    , m_pidX(
	     get(n, "PIDs/X/kp"),
	     get(n, "PIDs/X/kd"),
	     get(n, "PIDs/X/ki"),
	     get(n, "PIDs/X/minOutput"),
	     get(n, "PIDs/X/maxOutput"),
	     get(n, "PIDs/X/integratorMin"),
	     get(n, "PIDs/X/integratorMax"),
	     "x")
    , m_pidY(
	     get(n, "PIDs/Y/kp"),
	     get(n, "PIDs/Y/kd"),
	     get(n, "PIDs/Y/ki"),
	     get(n, "PIDs/Y/minOutput"),
	     get(n, "PIDs/Y/maxOutput"),
	     get(n, "PIDs/Y/integratorMin"),
	     get(n, "PIDs/Y/integratorMax"),
	     "y")
    , m_pidZ(
	     get(n, "PIDs/Z/kp"),
	     get(n, "PIDs/Z/kd"),
	     get(n, "PIDs/Z/ki"),
	     get(n, "PIDs/Z/minOutput"),
	     get(n, "PIDs/Z/maxOutput"),
	     get(n, "PIDs/Z/integratorMin"),
	     get(n, "PIDs/Z/integratorMax"),
	     "z")
    , m_pidYaw(
	       get(n, "PIDs/Yaw/kp"),
	       get(n, "PIDs/Yaw/kd"),
	       get(n, "PIDs/Yaw/ki"),
	       get(n, "PIDs/Yaw/minOutput"),
	       get(n, "PIDs/Yaw/maxOutput"),
	       get(n, "PIDs/Yaw/integratorMin"),
	       get(n, "PIDs/Yaw/integratorMax"),
	       "yaw")
    , m_state(Idle)
    , m_goal()
    , m_joy()
    , m_imu()
    , m_subscribeGoal()
    , m_subscribeJoy()
    , m_subscribeImu()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_thrust(0)
    , m_startZ(0)
    , v_x(-3, 3, 6.5, "vx")
    , v_y(-3, 3, 6.5, "vy")
    , v_z(-3, 3, 4.5, "vz")
    , v_r(-3, 3, 6.5, "vr")
    , v_s(-3, 3, 6.5, "vs")
    , filter_roll(-30, 30, 6.5, "roll")
    , filter_pitch(-30, 30, 6.5, "pitch")
    , velocity()
    , pendulum_v()
    , count(0)
    , IP(false)
    , lastDrone()
    , lastPendulum()
    , FirstIn(true)
  {
    ros::NodeHandle nh;
    m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
    m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_pubPose = nh.advertise<geometry_msgs::TransformStamped>("pose", 1);
    m_pubPendulum = nh.advertise<geometry_msgs::PoseStamped>("pendulum", 1);
    m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
    m_subscribeJoy  = nh.subscribe("joy", 1, &Controller::subscribeJoy, this);
    m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
    m_serviceLand = nh.advertiseService("land", &Controller::land, this);
  }

  void run(double frequency)
  {
    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
    ros::spin();
  }
    
  void initJoy()
  {
    for(int i = 0; i<8; i++)
      {
	m_joy.axes.push_back(0);
      }
    for(int i = 0; i<10; i++)
      {
	m_joy.buttons.push_back(0);
      }
    Joy_input.x = 0;
    Joy_input.y = 0;
    Joy_input.z = 0;
    Joy_input.yaw = 0;
  }

private:
  void goalChanged(
		   const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if(m_state != Landing)
      m_goal = *msg;
  }
    
  void subscribeJoy(
		    const sensor_msgs::JoyConstPtr& msg)
  {
    m_joy = *msg;
    Joy_input.x   =  m_joy.axes[4]*150.f;
    Joy_input.y   = -m_joy.axes[3]*150.f;
    Joy_input.z   =  m_joy.axes[1]*50000.f;
    Joy_input.yaw = -m_joy.axes[0]*30.f;
    if(m_joy.buttons[8] > 0.5)
      {
	IP = true;
      }
  }

  void subscribeImu(
		    const sensor_msgs::ImuConstPtr& msg)
  {
    m_imu = *msg;
  }

  bool takeoff(
	       std_srvs::Empty::Request& req,
	       std_srvs::Empty::Response& res)
  {
    ROS_INFO("Takeoff requested!");
    if (m_state != Automatic)
      m_state = TakingOff;

    tf::StampedTransform transform;
    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
    m_startZ = transform.getOrigin().z();

    return true;
  }

  bool land(
	    std_srvs::Empty::Request& req,
	    std_srvs::Empty::Response& res)
  {
    ROS_INFO("Landing requested!");
    m_state = Landing;

    return true;
  }

  void getTransform(
		    const std::string& sourceFrame,
		    const std::string& targetFrame,
		    tf::StampedTransform& result)
  {
    m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
  }

  void pidReset()
  {
    m_pidX.reset();
    m_pidY.reset();
    m_pidZ.reset();
    m_pidYaw.reset();
  }
  
  float saturate(float input, float max, float min)
  {
    return (input>max?max:input)<min?min:(input>max?max:input);
  }

  void LQR_controller(geometry_msgs::PoseStamped drone, float roll, float pitch, geometry_msgs::PoseStamped pendulum, geometry_msgs::Twist velocity, geometry_msgs::Twist pendulum_v, sensor_msgs::Imu m_imu, geometry_msgs::Twist& output)
  {
    if(IP){
    }else{
      pendulum.pose.position.x = 0;//transform_p.getOrigin().x();
      pendulum.pose.position.y = 0;//transform_p.getOrigin().y();
      pendulum_v.linear.x = 0;
      pendulum_v.linear.y = 0;
    }
    
    float z_gain = 1600, u_1 = 0, u_2 = 0, u_3 = 0;
    u_1 = 34.3981*pendulum.pose.position.y + 3.8094*pendulum_v.linear.y + 1.0*drone.pose.position.y + 1.2407*velocity.linear.y + 8.2222*roll;// - 6.*m_imu.angular_velocity.x;
    u_2 = -34.3981*pendulum.pose.position.x - 3.8094*pendulum_v.linear.x - 1.0*drone.pose.position.x - 1.2407*velocity.linear.x + 8.2222*pitch; //- 6.*m_imu.angular_velocity.y;
    //u_3 = z_gain*(1.0000*drone.pose.position.z + 1.5811*velocity.linear.z);
    //    0.0000    0.0000   38.6026    9.6007    0.0000    0.0000    1.0000    1.4634   -0.0000   12.6685    0.0000   -0.0000
    //  -38.6026   -9.6007   -0.0000   -0.0000   -1.0000   -1.4634   -0.0000   -0.0000   12.6685   -0.0000   -0.0000    0.0000
    //u_1 = 201.0942*pendulum.pose.position.y + 49.8522*pendulum_v.linear.y + 3.0*drone.pose.position.y + 3.4634*velocity.linear.y + 6.6685*roll;
    //u_2 = -201.0942*pendulum.pose.position.x - 49.8522*pendulum_v.linear.x - 3.0*drone.pose.position.x - 3.4634*velocity.linear.x + 6.6685*pitch;
    u_3 = z_gain*(3.1623*drone.pose.position.z + 3.3652*velocity.linear.z);


    output.angular.x = 57.*u_2;
    output.angular.y = 57.*u_1;
    output.linear.z = u_3;
  }

    float compensate_GE(float input, float height)
  {
    float GE_a = 0.4423;
    float GE_b = -18.0;
    float kG = GE_a*exp(GE_b*height)+1.0f;
    return input/sqrt(kG);
  }

  void iteration(const ros::TimerEvent& e)
  {
    float dt = e.current_real.toSec() - e.last_real.toSec();
    // My code here	
    switch(m_state)
      {
      case TakingOff:
	{
	  tf::StampedTransform transform;
	  m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
	  geometry_msgs::TransformStamped pose;
	  tf::transformStampedTFToMsg(transform, pose);
	  m_pubPose.publish(pose);

	  if (transform.getOrigin().z() > m_startZ + 0.035 || m_thrust > 30000)
	    {
	      pidReset();
	      m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
	      m_state = Automatic;
	      m_thrust = 0;
	    }
	  else
	    {
	      m_thrust += 10000 * dt;
	      geometry_msgs::Twist msg;
	      msg.linear.z = m_thrust;
	      msg.linear.y = 30;
	      m_pubNav.publish(msg);
	    }

	}
	break;
      case Landing:
	{
	  m_goal.pose.position.z -= 0.2*dt;
	  //m_goal.pose.position.z = m_startZ + 0.05;
	  tf::StampedTransform transform;
	  m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
	  geometry_msgs::TransformStamped pose;
	  tf::transformStampedTFToMsg(transform, pose);
	  m_pubPose.publish(pose);

	  if (transform.getOrigin().z() <= m_startZ + 0.05) {
	    m_state = Idle;
	    geometry_msgs::Twist msg;
	    m_pubNav.publish(msg);
	  }
	}
	// intentional fall-thru
      case Automatic:
	{

	  tf::StampedTransform transform,transform_p;
	  m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
	  m_listener.lookupTransform("quad_noROT", "/vicon/inv_pendulum/inv_pendulum", ros::Time(0), transform_p);
	  geometry_msgs::TransformStamped pose;
	  geometry_msgs::TransformStamped inv_pendulum_ts;
	  geometry_msgs::PoseStamped inv_pendulum;
	  tf::transformStampedTFToMsg(transform, pose);
	  tf::transformStampedTFToMsg(transform_p, inv_pendulum_ts);
	  inv_pendulum.pose.position.x = inv_pendulum_ts.transform.translation.x;
	  inv_pendulum.pose.position.y = inv_pendulum_ts.transform.translation.y;

	  m_pubPose.publish(pose);

	  geometry_msgs::PoseStamped targetWorld;
	  targetWorld.header.stamp = transform.stamp_;
	  targetWorld.header.frame_id = m_worldFrame;
	  targetWorld.pose = m_goal.pose;

	  geometry_msgs::PoseStamped targetDrone;
	  m_listener.transformPose(m_frame, targetWorld, targetDrone);

	  tfScalar roll, pitch, yaw;
	  tf::Matrix3x3(
			tf::Quaternion(
				       targetDrone.pose.orientation.x,
				       targetDrone.pose.orientation.y,
				       targetDrone.pose.orientation.z,
				       targetDrone.pose.orientation.w
				       )).getRPY(roll, pitch, yaw);

	  geometry_msgs::Twist msg;
	  //cout<<"Xerror: "<<targetDrone.pose.position.x<<endl;
	  //cout<<"Current yaw: "<<yaw<<endl;
	  msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x)   + Joy_input.x;
	  msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y) + Joy_input.y;
	  //cout<<targetDrone.pose.position.x<<"  "<<targetDrone.pose.position.y<<"  "<<targetDrone.pose.position.z<<endl;
	  targetDrone.pose.position.x *= -1.;
	  targetDrone.pose.position.y *= -1.;
	  //roll = -roll;
	  //pitch = -pitch;
	  //Calculate LPF

	  inv_pendulum.pose.position.x = -transform_p.getOrigin().x();
	  inv_pendulum.pose.position.y = -transform_p.getOrigin().y();
	  
	  //cout<<inv_pendulum.pose.position.x<<"    "<<inv_pendulum.pose.position.y<<endl;
	  // To avoid extreme initial velocity
	  if(FirstIn == true){
	    lastDrone = targetDrone;
	    lastPendulum = inv_pendulum;
	    FirstIn = false;
	  }
	  
	  velocity.linear.x = v_x.filter((targetDrone.pose.position.x-lastDrone.pose.position.x)/0.02);
	  velocity.linear.y = v_y.filter((targetDrone.pose.position.y-lastDrone.pose.position.y)/0.02);
	  velocity.linear.z = v_z.filter((targetDrone.pose.position.z-lastDrone.pose.position.z)/0.02);
	  pendulum_v.linear.x = v_r.filter((inv_pendulum.pose.position.x-lastPendulum.pose.position.x)/0.02);
	  pendulum_v.linear.y = v_s.filter((inv_pendulum.pose.position.y-lastPendulum.pose.position.y)/0.02);
	  roll = filter_roll.filter(roll);
	  pitch = filter_pitch.filter(pitch);
		
	  //LQR controller here
	  geometry_msgs::Twist output;
	  LQR_controller(targetDrone, roll, pitch, inv_pendulum, velocity, pendulum_v, m_imu, output);
	  
	  // For Debug
	  inv_pendulum.header.stamp = ros::Time::now();
	  inv_pendulum.pose.orientation.x = pendulum_v.linear.x;
	  inv_pendulum.pose.orientation.y = pendulum_v.linear.y;
	  m_pubPendulum.publish(inv_pendulum);
	  lastDrone = targetDrone;
	  lastPendulum = inv_pendulum;
	  //cout<<output.angular.x<<"        "<<output.angular.y<<endl;
	  //cout<<roll*57.32<<"   "<<pitch*57.32<<endl;
	  //output.angular.x = 0;
	  //output.angular.y = 0;
	  
	  msg.linear.x = saturate(output.angular.x, 1000, -1000);//60.*pow(-1, (count%50.0));// + Joy_input.x;
	  msg.linear.y = saturate(output.angular.y, 1000, -1000);//60.*pow(-1, count/50);// + Joy_input.y;
	  msg.linear.z = saturate(m_pidZ.update(0.0, targetDrone.pose.position.z, velocity.linear.z) + 10*fabs(roll)+10*fabs(pitch), 60000, 0);
	  //msg.linear.z = saturate(output + Joy_input.z, 60000, 0);
          #ifdef DIRECT_GE_COMPENSATE
	  msg.linear.z = compensate_GE(msg.linear.z, m_goal.pose.position.z);//pose.transform.translation.z+0.08);
          #endif
	  msg.angular.z = m_pidYaw.update(0.0, yaw)                      + Joy_input.yaw;
	  m_pubNav.publish(msg);


	}
	break;
      case Idle:
	{
	  tf::StampedTransform transform;
	  m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
	  geometry_msgs::TransformStamped pose;
	  tf::transformStampedTFToMsg(transform, pose);
	  m_pubPose.publish(pose);
	  geometry_msgs::Twist msg;
	  msg.linear.x  = Joy_input.x;
	  msg.linear.y  = Joy_input.y;
	  msg.linear.z  = Joy_input.z;
	  msg.angular.z = Joy_input.yaw;
	  m_pubNav.publish(msg);
	}
	break;
      }
    count++;
  }

private:

  enum State
    {
      Idle = 0,
      Automatic = 1,
      TakingOff = 2,
      Landing = 3,
    };

private:
  std::string m_worldFrame;
  std::string m_frame;
  ros::Publisher m_pubNav;
  ros::Publisher m_pubPose;
  ros::Publisher m_pubPendulum;
  tf::TransformListener m_listener;
  PID m_pidX;
  PID m_pidY;
  PID m_pidZ;
  PID m_pidYaw;
  State m_state;
  geometry_msgs::PoseStamped m_goal;
  sensor_msgs::Joy m_joy;
  sensor_msgs::Imu m_imu;
  ros::Subscriber m_subscribeGoal;
  ros::Subscriber m_subscribeJoy;
  ros::Subscriber m_subscribeImu;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  float m_thrust;
  float m_startZ;
  LOWPASS v_x, v_y, v_z, v_r, v_s, filter_roll, filter_pitch;
  geometry_msgs::Twist velocity, pendulum_v;
  int count;
  bool IP, FirstIn;
  geometry_msgs::PoseStamped lastDrone;
  geometry_msgs::PoseStamped lastPendulum;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.initJoy();
  controller.run(frequency);

  return 0;
}
