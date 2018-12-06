#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>


class LOWPASS
{
public:
    LOWPASS(
        float minOutput,
        float maxOutput,
	float CUTOFF,
        const std::string& name)
        : m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_CUTOFF(CUTOFF)
        , m_previousTime(ros::Time::now())
	, m_lastOutput(0)
	, m_pubVelocity()
	, m_pubFilterVelocity()
    {
	std::string filter_name = "Filter_" + name;
        ros::NodeHandle nh;
	m_pubVelocity = nh.advertise<std_msgs::Float32>(name, 1);
	m_pubFilterVelocity = nh.advertise<std_msgs::Float32>(filter_name, 1);
    }

    void reset()
    {
	m_lastOutput = 0;
        m_previousTime = ros::Time::now();
    }

    float filter(float input)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float CUTOFF = m_CUTOFF;
        float alpha = 1.0 / (1.0 / (2.0 * 3.1415926 * CUTOFF * dt) + 1.0);
	if (input>m_maxOutput)
	  {
	    input = m_maxOutput;
	  }
	else if(input<m_minOutput)
	  {
	    input = m_minOutput;
	  }
        float output = m_lastOutput + alpha * (input - m_lastOutput);
	//if(input < 20 && input > -20)
	//    m_lastOutput = output;
	m_previousTime = time;
	m_lastOutput = output;
	std_msgs::Float32 msg;
	msg.data = output;
	m_pubFilterVelocity.publish(msg);
	return m_lastOutput;
    }

private:
    float m_minOutput;
    float m_maxOutput;
    float m_lastOutput;
    float m_CUTOFF;
    ros::Time m_previousTime;
    ros::Publisher m_pubVelocity;
    ros::Publisher m_pubFilterVelocity;
};
