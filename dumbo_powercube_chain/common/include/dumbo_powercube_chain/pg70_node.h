/*
 * pg70_node.h
 *
 *  Created on: Jan 25, 2013
 *      Author: fevb
 */

#ifndef PG70_NODE_H_
#define PG70_NODE_H_

#include <ros/ros.h>
#include <cob_srvs/Trigger.h>
#include <dumbo_powercube_chain/PG70Gripper.h>
#include <dumbo_powercube_chain/PowerCubeCtrl.h>
#include <dumbo_powercube_chain/PowerCubeCtrlParams.h>
#include <dumbo_srvs/ClosePG70Gripper.h>
#include <cob_srvs/Trigger.h>
#include <brics_actuator/JointPositions.h>

// Requires the LWA to be initialized first...

class PG70Node {
public:
	ros::NodeHandle n_;
	bool initialized_;
	ros::Time last_publish_time_;
	bool error_;
	std::string error_msg_;
	double frequency_;

	// namespace to put the node on (default == /PG70_controller)
	PG70Node(std::string name = "/PG70_controller");
	virtual ~PG70Node();

	void getROSParameters();

	void getRobotDescriptionParameters();

	// the PG70 must be initialized after the arm!!!!
	bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    bool srvCallback_Disconnect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);

	bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );

	void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg);

	bool srvCallback_CloseGripper(dumbo_srvs::ClosePG70Gripper::Request &req, dumbo_srvs::ClosePG70Gripper::Response &res);

	void publishState(bool update=true);

	double getFrequency(){ return frequency_;}

private:

	// for controlling the parallel gripper PG70
	PG70Gripper* pg70_ctrl_;
	PowerCubeCtrlParams* pg70_params_;

	ros::Publisher topicPub_JointState_;
	ros::Subscriber topicSub_CommandPos_;

	ros::ServiceServer srvServer_Init_;
    ros::ServiceServer srvServer_Disconnect_;
	ros::ServiceServer srvServer_CloseGripper_;
	ros::ServiceServer srvServer_Recover_;

};

#endif /* PG70_NODE_H_ */
