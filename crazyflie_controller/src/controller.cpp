#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <cmath>

#include "pid.hpp"
#include "LPfilter.h"

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
        , m_subscribeGoal()
        , m_subscribeJoy()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
      , x_filter(20, 0.05)
      , y_filter(20, 0.05)
      , z_filter(20, 0.05)
      , yaw_filter(20, 0.05)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	m_pubPose = nh.advertise<geometry_msgs::TransformStamped>("pose", 1);
	m_pubError = nh.advertise<geometry_msgs::Vector3>("error", 1);
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
        Joy_input.x   =  m_joy.axes[4]*30.f;
        Joy_input.y   = -m_joy.axes[3]*30.f;
        Joy_input.z   =  m_joy.axes[1]*50000.f;
        Joy_input.yaw = -m_joy.axes[0]*30.f;
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
      if(input > max)
	return max;
      else if(input < min)
	return min;
      else
	return input;
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

                if (transform.getOrigin().z() > m_startZ + 0.02 || m_thrust > 40000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
		    cout<<"Successfully takeoff!"<<endl;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
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
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
		geometry_msgs::TransformStamped pose;
		tf::transformStampedTFToMsg(transform, pose);
		m_pubPose.publish(pose);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);
		//Overwrite the xyz error
	        targetDrone.pose.position.x = m_goal.pose.position.x - pose.transform.translation.x;
		targetDrone.pose.position.y = m_goal.pose.position.y - pose.transform.translation.y;
		targetDrone.pose.position.z = m_goal.pose.position.z - pose.transform.translation.z;

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
		//cout<<"P error: "<<targetDrone.pose.position.x<<" "<<targetDrone.pose.position.y<<" "<<targetDrone.pose.position.z<<endl;
		//cout<<"Current yaw: "<<yaw<<endl;
		// Saturate the maximum error
		targetDrone.pose.position.x = x_filter.update(saturate(targetDrone.pose.position.x, 1.2, -1.2));
		targetDrone.pose.position.y = y_filter.update(saturate(targetDrone.pose.position.y, 1.2, -1.2));
		targetDrone.pose.position.z = z_filter.update(saturate(targetDrone.pose.position.z, 0.8, -0.8));
		yaw = yaw_filter.update(saturate(yaw, 20, -20));

		
		geometry_msgs::Vector3 error;
		error.x = targetDrone.pose.position.x;
		error.y = targetDrone.pose.position.y;
		error.z = targetDrone.pose.position.z;
		m_pubError.publish(error);

                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x)   + Joy_input.x;
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y) + Joy_input.y;
                msg.linear.z = saturate(m_pidZ.update(0.0, targetDrone.pose.position.z) + Joy_input.z, 60000, 0);
                #ifdef DIRECT_GE_COMPENSATE
                msg.linear.z = compensate_GE(msg.linear.z, m_goal.pose.position.z);//pose.transform.translation.z+0.08);
                #endif
                msg.angular.z = saturate(m_pidYaw.update(0.0, yaw)                      + Joy_input.yaw, 20, -20);
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
    ros::Publisher m_pubError;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    sensor_msgs::Joy m_joy;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeJoy;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
  LPfilter x_filter;
  LPfilter y_filter;
  LPfilter z_filter;
  LPfilter yaw_filter;

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
