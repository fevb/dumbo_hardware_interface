/*
 * test_movevel.cpp
 *
 *  Created on: Aug 14, 2012
 *      Author: fevb
 */





#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_movevel");
	ros::NodeHandle n_;
	ros::Publisher joint_vel_pub_ = n_.advertise<brics_actuator::JointVelocities>("command_vel", 1);;
	int DOF;

	// get JointNames from parameter server
	XmlRpc::XmlRpcValue JointNames_param_;
	std::vector<std::string> JointNames_;
	ROS_INFO("getting JointNames from parameter server");
	if (n_.hasParam("joint_names"))
	{
		n_.getParam("joint_names", JointNames_param_);
	}
	else
	{
		ROS_ERROR("Parameter joint_names not set");
	}
	JointNames_.resize(JointNames_param_.size());
	for (int i = 0; i<JointNames_param_.size(); i++ )
	{
		JointNames_[i] = (std::string)JointNames_param_[i];
	}
	DOF = JointNames_param_.size();


	brics_actuator::JointVelocities target_joint_vel;
	target_joint_vel.velocities.resize(DOF);
	for(int i=0; i<DOF; i++)
	{
		target_joint_vel.velocities[i].joint_uri = JointNames_[i].c_str();
		target_joint_vel.velocities[i].unit = "rad";
		target_joint_vel.velocities[i].value = 0;
	}

	target_joint_vel.velocities[6].value = 5*3.1416/180;

	ros::Time t_start = ros::Time::now();
	ros::Duration time_from_start = ros::Time::now() - t_start;
	ros::Rate loop_rate(20);

	while(ros::ok() && (time_from_start.toSec()<1.0))
	{
		time_from_start = ros::Time::now()-t_start;
		joint_vel_pub_.publish(target_joint_vel);
		loop_rate.sleep();
	}



	return 0;
}
