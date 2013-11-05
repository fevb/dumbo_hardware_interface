/*
 *  pg70_node
 *
 *  Created on: Jan 25, 2013
 *  Authors:   Francisco Viña 
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
