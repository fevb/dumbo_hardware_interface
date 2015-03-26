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
#include <diagnostic_msgs/DiagnosticArray.h>

PG70Node::PG70Node(ros::NodeHandle nh,
                   boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                   boost::shared_ptr<canHandle> CAN_handle) :
    n_(nh)
{

	// TODO Auto-generated constructor stub
    pg70_params_.reset(new PowerCubeCtrlParams());
    pg70_ctrl_.reset(new PG70Gripper(pg70_params_, CAN_mutex, CAN_handle));

	topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    topicPub_ControllerState_ = n_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
    topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &PG70Node::topicCallbackCommandPos, this);
    topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &PG70Node::topicCallbackCommandVel, this);

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
    XmlRpc::XmlRpcValue ArmNameXmlRpc;
    std::string arm_name;
    if (n_.hasParam("arm_name"))
	{
        n_.getParam("arm_name", ArmNameXmlRpc);
	}

	else
	{
        ROS_ERROR("Parameter arm_name not set in /PG70_controller, shutting down node...");
		n_.shutdown();
	}

    arm_name = (std::string)(ArmNameXmlRpc);
    if((arm_name!="left") && (arm_name!="right"))
	{
        ROS_ERROR("Invalid arm_name parameter in /PG70_controller, shutting down node... ");
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

    pg70_params_->Init(CanBaudrate, ModulIDs);
    pg70_params_->setArmName(arm_name);
    pg70_params_->SetJointNames(JointNames);
    pg70_params_->SetMaxAcc(MaxAccelerations);
    pg70_params_->SetSerialNumber((unsigned long int) SerialNumber);

    // initialize joint state messages
    unsigned int n_ros_joints = pg70_params_->GetJointNames().size();
    joint_state_msg_.name = pg70_params_->GetJointNames();
    joint_state_msg_.position = std::vector<double>(n_ros_joints, 0.0);
    joint_state_msg_.velocity = std::vector<double>(n_ros_joints, 0.0);

    unsigned int dof = pg70_params_->GetDOF();
    controller_state_msg_.joint_names = std::vector<std::string>(dof, "pg70");
    controller_state_msg_.actual.positions = std::vector<double>(dof, 0.0);
    controller_state_msg_.actual.velocities = std::vector<double>(dof, 0.0);
    controller_state_msg_.actual.accelerations = std::vector<double>(dof, 0.0);
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
          LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower*2.0;

		  // Get gripper upper limit
          UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper*2.0;

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

        if(pg70_ctrl_->init())
		{
			initialized_ = true;
			res.success.data = true;
			ROS_INFO("...initializing PG70 gripper successful");
			publishState(true);
		}

		else
		{
            ROS_ERROR("Couldn't initialize PG70 gripper on %s arm.", pg70_params_->getArmName().c_str());
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
        if (pg70_ctrl_->recover())
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
void PG70Node::topicCallbackCommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
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
        std::string unit = "m";


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
        cmd_gripper_pos = msg->positions[0].value;


		/// command position to gripper
        if (!pg70_ctrl_->movePos(cmd_gripper_pos))
		{
            ROS_ERROR("Error executing gripper position command in %s arm.", (pg70_params_->getArmName()).c_str());
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
        ROS_ERROR("Skipping gripper positions command: no gripper on %s arm.", (pg70_params_->getArmName()).c_str());
	}

}

void PG70Node::topicCallbackCommandVel(const brics_actuator::JointVelocities::ConstPtr &msg)
{
    if (initialized_ && (pg70_params_->GetDOF()>0))
    {
        //		  PowerCubeCtrl::PC_CTRL_STATUS status;
        //		  std::vector<std::string> errorMessages;
        //		  pc_ctrl_->getStatus(status, errorMessages);

        std::vector<std::string> JointNames = pg70_params_->GetJointNames();
        double cmd_gripper_vel;
        std::string unit = "m/s";


        /// parse velocity
        /// check joint name
        if (msg->velocities[0].joint_uri != JointNames[0])
        {
            ROS_ERROR("Skipping command: Received gripper joint name %s doesn't match expected joint name %s.",
                        msg->velocities[0].joint_uri.c_str(), JointNames[0].c_str());
            return;
        }

        /// check unit
        if (msg->velocities[0].unit != unit)
        {
            ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.", msg->velocities[0].unit.c_str(), unit.c_str());
            return;
        }

        /// if all checks are successful, parse the position value for the gripper
        ROS_DEBUG("Parsing position %f for joint %s", msg->velocities[0].value, JointNames[0].c_str());
        cmd_gripper_vel = msg->velocities[0].value;


        /// command velocity to gripper
        if (!pg70_ctrl_->moveVel(cmd_gripper_vel))
        {
            ROS_ERROR("Error executing gripper velocity command in %s arm.", (pg70_params_->getArmName()).c_str());
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
        ROS_ERROR("Skipping gripper positions command: no gripper on %s arm.", (pg70_params_->getArmName()).c_str());
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
        ROS_ERROR("Error trying to close gripper on %s arm", pg70_params_->getArmName().c_str());
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
        if(update || pg70_ctrl_->executingPosCommand())
		{
			pg70_ctrl_->updateStates();
			ROS_DEBUG("Updated PG70 gripper");
		}

        joint_state_msg_.header.stamp = ros::Time::now();
        unsigned int n_ros_joints = pg70_params_->GetJointNames().size();
        for(unsigned int i=0; i<n_ros_joints; i++)
        {
            joint_state_msg_.position[i] = pg70_ctrl_->getPositions().at(0)/2.0;
		}

        // controller state message
        controller_state_msg_.header.stamp = joint_state_msg_.header.stamp;
        controller_state_msg_.actual.positions[0] = pg70_ctrl_->getPositions().at(0);

        // publish joint and controller states
        topicPub_JointState_.publish(joint_state_msg_);
        topicPub_ControllerState_.publish(controller_state_msg_);
        last_publish_time_ = joint_state_msg_.header.stamp;
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


