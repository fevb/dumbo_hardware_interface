/*
 * sdh_node.h
 *
 *  Created on: Jan 25, 2013
 *      Author: fevb
 */
#ifndef SDHNODE_h_
#define SDHNODE_h_


// standard includes
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <schunk_sdh/TactileSensor.h>
#include <schunk_sdh/TactileMatrix.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// ROS diagnostic msgs
#include <diagnostic_msgs/DiagnosticArray.h>

// external includes
#include <schunk_sdh/sdh.h>
#include <schunk_sdh/dsa.h>

// dumbo srvs
#include <dumbo_srvs/GetSDHMotorCurrents.h>
#include <dumbo_srvs/SetSDHMotorCurrents.h>


/*!
 * \brief Implementation of ROS node for sdh.
 *
 * Offers actionlib and direct command interface.
 */
class SdhNode
{
	//	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	//	std::string action_name_;
public:
	/// create a handle for this node, initialize node
	ros::NodeHandle nh_;

private:
	// declaration of topics to publish
	ros::Publisher topicPub_JointState_;
	ros::Publisher topicPub_ControllerState_;
	ros::Publisher topicPub_TactileSensor_;
	ros::Publisher topicPub_Diagnostics_;

	// topic subscribers
	ros::Subscriber subSetVelocitiesRaw_;
	ros::Subscriber subSetVelocities_;

	// service servers
	ros::ServiceServer srvServer_Init_;
	ros::ServiceServer srvServer_Stop_;
	ros::ServiceServer srvServer_Recover_;
	ros::ServiceServer srvServer_SetOperationMode_;
	ros::ServiceServer srvServer_Shutdown_;

	ros::ServiceServer srvServer_GetMotorCurrents_;
	ros::ServiceServer srvServer_SetMotorCurrents_;

	ros::ServiceServer srvServer_OpenHand_;


	// actionlib server
	//actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	std::string action_name_;

	// service clients
	//--

	// other variables
	SDH::cSDH *sdh_;
	SDH::cDSA *dsa_;
	std::vector<SDH::cSDH::eAxisState> state_;

	std::string sdhdevicetype_;
	std::string sdhdevicestring_;
	int sdhdevicenum_;
	std::string dsadevicestring_;
	int dsadevicenum_;
	int baudrate_, id_read_, id_write_;
	double timeout_;
	double frequency_;

	bool isInitialized_;
	bool isDSAInitialized_;
	bool isError_;
	int DOF_;
	double pi_;

	trajectory_msgs::JointTrajectory traj_;

	std::vector<std::string> joint_names_;
	std::vector<int> axes_;
	std::vector<double> actualAngles_;
	std::vector<double> actualVelocities_;
	std::vector<double> targetAngles_; // in degrees
	std::vector<double> velocities_; // in rad/s

	// for getting the temperature
	std::vector<int> sensors_temp_index_;
	std::vector<double> temp_; // in degrees celsius

	std::vector<double> motor_currents_;

	bool hasNewGoal_;
	std::string operationMode_;

public:
	/// create a handle for this node, initialize node
	ros::Time last_publish_time_;

	SdhNode(std::string name);
	~SdhNode();

	bool init();

	void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

	void topicCallback_setVelocitiesRaw(const std_msgs::Float32MultiArrayPtr& velocities);

	bool parseDegFromJointValue(const brics_actuator::JointValue& val, double &deg_val);

	void topicCallback_setVelocities(const brics_actuator::JointVelocities::ConstPtr& msg);

	// the SDH must be initialized after the arm!!!
	bool srvCallback_Init(cob_srvs::Trigger::Request &req,
			cob_srvs::Trigger::Response &res );

	bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
			cob_srvs::Trigger::Response &res );

	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
			cob_srvs::Trigger::Response &res );

	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
			cob_srvs::SetOperationMode::Response &res );

	bool srvCallback_Shutdown(cob_srvs::Trigger::Request &req,
				cob_srvs::Trigger::Response &res );

	bool srvCallback_GetMotorCurrents(dumbo_srvs::GetSDHMotorCurrents::Request &req,
			dumbo_srvs::GetSDHMotorCurrents::Response &res);

	bool srvCallback_SetMotorCurrents(dumbo_srvs::SetSDHMotorCurrents::Request &req,
			dumbo_srvs::SetSDHMotorCurrents::Response &res);

	bool srvCallback_OpenHand(cob_srvs::Trigger::Request &req,
					cob_srvs::Trigger::Response &res );

	void updateSdh(bool update=true);
	void updateDsa();

	double getFrequency(){return frequency_;}


};

#endif /* SDH_NODE_H_ */
