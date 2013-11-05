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


#include <dumbo_powercube_chain/pg70_node.h>
#include <urdf/model.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>

PG70Node::PG70Node(std::string name) {

	n_ = ros::NodeHandle(name);
	// TODO Auto-generated constructor stub
	pg70_params_ = new PowerCubeCtrlParams();
	pg70_ctrl_ = new PG70Gripper(pg70_params_);

	topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
	topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &PG70Node::topicCallback_CommandPos, this);

	srvServer_Init_ = n_.advertiseService("init", &PG70Node::srvCallback_Init, this);
    srvServer_Disconnect_ = n_.advertiseService("disconnect", &PG70Node::srvCallback_Disconnect, this);
	srvServer_Recover_ = n_.advertiseService("recover", &PG70Node::srvCallback_Recover, this);
	srvServer_CloseGripper_ = n_.advertiseService("close_gripper", &PG70Node::srvCallback_CloseGripper, this);
	initialized_ = false;
	last_publish_time_ = ros::Time::now();
}

PG70Node::~PG70Node() {
	// TODO Auto-generated destructor stub
	ROS_INFO("PG70 Device closed!");
	delete pg70_ctrl_;
	delete pg70_params_;
}



void PG70Node::getROSParameters()
{

	/// get CanBaudrate
	int CanBaudrate;
	if (n_.hasParam("can_baudrate"))
	{
		n_.getParam("can_baudrate", CanBaudrate);
	}
	else
	{
		ROS_ERROR("Parameter can_baudrate not set in /PG70_controller, shutting down node...");
		n_.shutdown();
	}

	/// Get arm selection parameter (left or right arm)
	XmlRpc::XmlRpcValue ArmSelectXmlRpc;
	std::string ArmSelect;
	if (n_.hasParam("arm_select"))
	{
		n_.getParam("arm_select", ArmSelectXmlRpc);
	}

	else
	{
		ROS_ERROR("Parameter arm_select not set in /PG70_controller, shutting down node...");
		n_.shutdown();
	}

	ArmSelect = (std::string)(ArmSelectXmlRpc);
	if((ArmSelect!="left") && (ArmSelect!="right"))
	{
		ROS_ERROR("Invalid arm_select parameter in /PG70_controller, shutting down node... ");
		n_.shutdown();
	}


	/// get Modul IDs
	XmlRpc::XmlRpcValue ModulIDsXmlRpc;
	std::vector<int> ModulIDs;
	if (n_.hasParam("module_id"))
	{
		n_.getParam("module_id", ModulIDsXmlRpc);
	}

	else
	{
		ROS_ERROR("Parameter modul_id not set in /PG70_controller, shutting down node...");
		n_.shutdown();
	}

	/// Resize and assign of values to the ModulIDs
	ModulIDs.resize(ModulIDsXmlRpc.size());
	for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
	{
		ModulIDs[i] = (int)ModulIDsXmlRpc[i];
	}

    /// Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;
    if (n_.hasParam("joint_names"))
    {
    	n_.getParam("joint_names", JointNamesXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter joint_names not set in /PG70_controller, shutting down node...");
    	n_.shutdown();
    }

    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
    	JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }


    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (n_.hasParam("max_acceleration"))
    {
    	n_.getParam("max_acceleration", MaxAccelerationsXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter max_acceleration not set in /PG70_controller, shutting down node...");
    	n_.shutdown();
    }

    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
    	MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// get Gripper Max acceleration
    int SerialNumber;
    if (n_.hasParam("serial_number"))
    {
    	n_.getParam("serial_number", SerialNumber);
    }

    else
    {
    	ROS_ERROR("Parameter serial_number not set in /PG70_controller, shutting down node...");
    	n_.shutdown();
    }

    // get publish frequency
    n_.param("frequency", frequency_, 5.0);

    pg70_params_->Init(CanBaudrate, ModulIDs);
    pg70_params_->SetArmSelect(ArmSelect);
    pg70_params_->SetJointNames(JointNames);
    pg70_params_->SetMaxAcc(MaxAccelerations);
    pg70_params_->SetSerialNumber((unsigned long int) SerialNumber);
}


/*!
 * \brief Gets parameters from the robot_description and configures the powercube_chain.
 */
void PG70Node::getRobotDescriptionParameters()
{

	  /// Get robot_description from ROS parameter server
	  std::string param_name = "robot_description";
	  std::string full_param_name;
	  std::string xml_string;

	  n_.searchParam(param_name, full_param_name);
	  if (n_.hasParam(full_param_name))
	  {
		  n_.getParam(full_param_name.c_str(), xml_string);
	  }

	  else
	  {
		  ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
		  n_.shutdown();
	  }

	  if (xml_string.size() == 0)
	  {
		  ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
		  n_.shutdown();
	  }
	  ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

	  /// Get urdf model out of robot_description
	  urdf::Model model;
	  if (!model.initString(xml_string))
	  {
		  ROS_ERROR("Failed to parse urdf file");
		  n_.shutdown();
	  }
	  ROS_DEBUG("Successfully parsed urdf file");


	  // Get gripper params
	  std::vector<std::string> JointNames = pg70_params_->GetJointNames();
	  std::vector<double> LowerLimits(JointNames.size());
	  std::vector<double> UpperLimits(JointNames.size());
	  std::vector<double> MaxVel(JointNames.size());
	  for(unsigned int i=0; i<JointNames.size(); i++ )
	  {
		  // Get gripper lower limit
		  LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;

		  // Get gripper upper limit
		  UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;

		  // Get gripper max vel
		  MaxVel[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
	  }


	  pg70_params_->SetLowerLimits(LowerLimits);
	  pg70_params_->SetUpperLimits(UpperLimits);
	  pg70_params_->SetMaxVel(MaxVel);
}

/*!
 * \brief Executes the service callback for init.
 * Init service for the arm must be called first !!!
 * Connects to the hardware and initialized it.
 * \param req Service request
 * \param res Service response
 */
bool PG70Node::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
	if (!initialized_)
	{
		ROS_INFO("Initializing PG70 gripper...");

		if(pg70_ctrl_->Init())
		{
			initialized_ = true;
			res.success.data = true;
			ROS_INFO("...initializing PG70 gripper successful");
			publishState(true);
		}

		else
		{
			ROS_ERROR("Couldn't initialize PG70 gripper on %s arm.", pg70_params_->GetArmSelect().c_str());
			initialized_ = false;
			res.success.data = false;
		}

	}

	else
	{
		res.success.data = true;
		res.error_message.data = "PG70 gripper already initialized";
		ROS_WARN("...initializing PG70 gripper not successful. error: %s",res.error_message.data.c_str());
	}



	return true;
}

bool PG70Node::srvCallback_Disconnect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
    if (!initialized_)
    {
        ROS_WARN("PG70 gripper already switched off");
        res.success.data = false;
        res.error_message.data = "PG70 gripper already switched off";
    }

    else
    {
        initialized_ = false;
        res.success.data = true;
        ROS_INFO("Shutting down PG70 gripper");
    }


    return true;
}




/*!
 * \brief Executes the service callback for recover.
 *
 * Recovers the hardware after an emergency stop.
 * \param req Service request
 * \param res Service response
 */
bool PG70Node::srvCallback_Recover(cob_srvs::Trigger::Request &req,
		cob_srvs::Trigger::Response &res )
{
	ROS_INFO("Recovering PG70 Gripper...");
	if (initialized_)
	{
		/// stopping all arm movements
		if (pg70_ctrl_->Recover())
		{
			error_ = false;
			error_msg_ = "";
			res.success.data = true;
			ROS_INFO("...recovering PG70 gripper successful.");
		}
		else
		{
			res.success.data = false;
			error_ = true;
			error_msg_ = pg70_ctrl_->getErrorMessage();
			res.error_message.data = pg70_ctrl_->getErrorMessage();
			ROS_ERROR("...recovering PG70 gripper not successful. error: %s", res.error_message.data.c_str());
		}
	}

	else
	{
		res.success.data = false;
		res.error_message.data = "PG70 gripper not initialized";
		ROS_ERROR("...recovering PG70 gripper not successful. error: %s",res.error_message.data.c_str());
	}

	return true;
}


/*!
 * \brief Executes the callback from the command_pos topic.
 *
 * Set the current position target.
 * \param msg JointPositions
 */
void PG70Node::topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
{
	// ROS_WARN("Received new position command. Skipping command: Position commands currently not implemented");
	ROS_DEBUG("Received new gripper position command");
	if (initialized_ && (pg70_params_->GetDOF()>0))
	{
		//		  PowerCubeCtrl::PC_CTRL_STATUS status;
		//		  std::vector<std::string> errorMessages;
		//		  pc_ctrl_->getStatus(status, errorMessages);

		std::vector<std::string> JointNames = pg70_params_->GetJointNames();
		double cmd_gripper_pos;
		std::string unit = "mm";


		/// parse positions
		/// check joint name
		if (msg->positions[0].joint_uri != JointNames[0])
		{
			ROS_ERROR("Skipping command: Received gripper joint name %s doesn't match expected joint name %s.",msg->positions[0].joint_uri.c_str(),JointNames[0].c_str());
			return;
		}

		/// check unit
		if (msg->positions[0].unit != unit)
		{
			ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",msg->positions[0].unit.c_str(), unit.c_str());
			return;
		}

		/// if all checks are successful, parse the position value for the gripper
		ROS_DEBUG("Parsing position %f for joint %s",msg->positions[0].value, JointNames[0].c_str());
		cmd_gripper_pos = msg->positions[0].value/1000;


		/// command position to gripper
		if (!pg70_ctrl_->MovePos(cmd_gripper_pos))
		{
			ROS_ERROR("Error executing gripper position command in %s arm.", (pg70_params_->GetArmSelect()).c_str());
			//			  error_ = true;
			//			  error_msg_ = pc_ctrl_->getErrorMessage();
			//			  ROS_ERROR("Skipping command: %s",pc_ctrl_->getErrorMessage().c_str());//*** have to fix this
			return;
		}
		publishState(false);
		ROS_DEBUG("Executed gripper position command");
	}

	else if (!initialized_)
	{
		ROS_WARN("Skipping gripper position command: no gripper or not initialized");
	}

	else if (pg70_params_->GetDOF()==0)
	{
		ROS_ERROR("Skipping gripper positions command: no gripper on %s arm.", (pg70_params_->GetArmSelect()).c_str());
	}

}

bool PG70Node::srvCallback_CloseGripper(dumbo_srvs::ClosePG70Gripper::Request &req, dumbo_srvs::ClosePG70Gripper::Response &res)
{
	double target_vel = req.target_vel;
	double current_limit = req.current_limit;
	if(pg70_ctrl_->CloseGripper(target_vel, current_limit))
	{
		res.success = true;
		publishState(false);
	}

	else
	{
		ROS_ERROR("Error trying to close gripper on %s arm", pg70_params_->GetArmSelect().c_str());
		res.success = false;
	}

	return true;
}

void PG70Node::publishState(bool update)
{
	ROS_DEBUG("publish PG70 state");
	if (initialized_)
	{
		ROS_DEBUG("publish PG70 state -- initialized");

		// only update while in motion or specified by the parameter update
		if(update || pg70_ctrl_->inMotion())
		{
			pg70_ctrl_->updateStates();
			ROS_DEBUG("Updated PG70 gripper");
		}

		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();

		std::vector<std::string> JointNames = pg70_params_->GetJointNames();
		for(unsigned int i=0; i<JointNames.size(); i++)
		{
			joint_state_msg.name.push_back(JointNames[i].c_str());
			joint_state_msg.position.push_back(pg70_ctrl_->getPositions().at(0));
			joint_state_msg.velocity.push_back(0);
		}


		/// publishing joint and controller states on topic
		topicPub_JointState_.publish(joint_state_msg);
		last_publish_time_ = joint_state_msg.header.stamp;

	}

	// if not initialized publish zero angle + vel messages
	else
	{
		ROS_DEBUG("Publish state -- not initialized");
		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();

		std::vector<std::string> JointNames = pg70_params_->GetJointNames();
		for(unsigned int i=0; i<JointNames.size(); i++)
		{
			joint_state_msg.name.push_back(JointNames[i].c_str());
			joint_state_msg.position.push_back(0.0);
			joint_state_msg.velocity.push_back(0.0);
		}


		//		  ROS_ERROR("PUBLISHING");
		topicPub_JointState_.publish(joint_state_msg);
		last_publish_time_ = joint_state_msg.header.stamp;


	}
	ROS_DEBUG("PG70 joint states published");

}


