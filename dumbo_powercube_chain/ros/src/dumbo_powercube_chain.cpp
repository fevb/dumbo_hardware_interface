/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: schunk_modular_robotics
 * \note
 *   ROS stack name: schunk_modular_robotics
 * \note
 *   ROS package name: dumbo_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Dec 2010
 *
 * \brief
 *   Implementation of ROS node for powercube_chain.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####
//##################

// standard includes
// --
#include <string>
// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>


// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// own includes
#include <dumbo_powercube_chain/PowerCubeCtrl.h>
#include <dumbo_powercube_chain/PowerCubeCtrlParams.h>

// gripper nodes
#include <dumbo_powercube_chain/pg70_node.h>
#include <dumbo_powercube_chain/sdh_node.h>
#include <boost/scoped_ptr.hpp>

/*!
 * \brief Implementation of ROS node for powercube_chain.
 *
 * Offers velocity and position interface.
 */
class PowerCubeChainNode
{

public:


  /// declaration of topics to publish
  ros::Publisher topicPub_JointState_;
  ros::Publisher topicPub_ControllerState_;

  /// declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicSub_CommandPos_;
  ros::Subscriber topicSub_CommandVel_;

  /// declaration of service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Disconnect_;
  ros::ServiceServer srvServer_SetOperationMode_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceServer srvServer_Recover_;

  /// member variables
  bool initialized_;
  bool stopped_;
  bool error_;
  std::string error_msg_;
  ros::Time last_publish_time_;
  
  std::string operation_mode_;


  ///Constructor
  PowerCubeChainNode(ros::NodeHandle nh,
                     boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                     boost::shared_ptr<canHandle> CAN_handle) :
      n_(nh)
  {

    pc_params_.reset(new PowerCubeCtrlParams());
    pc_ctrl_.reset(new PowerCubeCtrl(pc_params_, CAN_mutex, CAN_handle));

    /// implementation of topics to publish
    topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    topicPub_ControllerState_ = n_.advertise<control_msgs::JointTrajectoryControllerState> ("state", 1);

    /// implementation of topics to subscribe
    topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &PowerCubeChainNode::topicCallback_CommandPos, this);
    topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &PowerCubeChainNode::topicCallback_CommandVel, this);

    /// implementation of service servers
    srvServer_Init_ = n_.advertiseService("init", &PowerCubeChainNode::srvCallback_Init, this);
    srvServer_Disconnect_ = n_.advertiseService("disconnect", &PowerCubeChainNode::srvCallback_Disconnect, this);
    srvServer_Stop_ = n_.advertiseService("stop_arm", &PowerCubeChainNode::srvCallback_Stop, this);
    srvServer_Recover_ = n_.advertiseService("recover", &PowerCubeChainNode::srvCallback_Recover, this);
    srvServer_SetOperationMode_ = n_.advertiseService("set_operation_mode", &PowerCubeChainNode::srvCallback_SetOperationMode, this);

    initialized_ = false;
    stopped_ = true;
    error_ = false;
    last_publish_time_ = ros::Time::now();
  }
  
  /// Destructor
  ~PowerCubeChainNode()
  {
      bool closed = pc_ctrl_->Close();
  }

  /*!
   * \brief Gets parameters from the ROS parameter server and configures the powercube_chain.
   */
  void getROSParameters()
  {


    /// get CanBaudrate
    int CanBaudrate;
    if (n_.hasParam("can_baudrate"))
    {
    	n_.getParam("can_baudrate", CanBaudrate);
    }
    else
    {
    	ROS_ERROR("Parameter can_baudrate not set, shutting down node...");
    	n_.shutdown();
    }

    /// get Modul IDs
    XmlRpc::XmlRpcValue ModulIDsXmlRpc;
    std::vector<int> ModulIDs;
    if (n_.hasParam("module_ids"))
    {
    	n_.getParam("module_ids", ModulIDsXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter module_ids not set, shutting down node...");
    	n_.shutdown();
    }
    
    /// Resize and assign of values to the ModulIDs
    ModulIDs.resize(ModulIDsXmlRpc.size());
    for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
    {
    	ModulIDs[i] = (int)ModulIDsXmlRpc[i];
    }

    /// Initialize parameters
    pc_params_->Init(CanBaudrate, ModulIDs);

    /// Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;
    if (n_.hasParam("joint_names"))
    {
    	n_.getParam("joint_names", JointNamesXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter joint_names not set, shutting down node...");
    	n_.shutdown();
    }
    
    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
    	JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)JointNames.size() != pc_params_->GetDOF())
    {
    	ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
    	n_.shutdown();
    }
    pc_params_->SetJointNames(JointNames);


    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (n_.hasParam("max_accelerations"))
    {
    	n_.getParam("max_accelerations", MaxAccelerationsXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter max_accelerations not set, shutting down node...");
    	n_.shutdown();
    }
    
    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
    	MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)MaxAccelerations.size() != pc_params_->GetDOF())
    {
    	ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
    	n_.shutdown();
    }
    pc_params_->SetMaxAcc(MaxAccelerations);

    /// Get horizon
    double Horizon;
    if (n_.hasParam("horizon"))
    {
    	n_.getParam("horizon", Horizon);
    }

    else
    {
    	/// Horizon in sec
    	Horizon = 0.05;
    	ROS_WARN("Parameter horizon not available, setting to default value: %f sec", Horizon);
    }
    pc_ctrl_->setHorizon(Horizon);


    /// Get arm selection parameter (left or right arm)
    XmlRpc::XmlRpcValue ArmSelectXmlRpc;
    std::string ArmSelect;
    if (n_.hasParam("arm_select"))
    {
    	n_.getParam("arm_select", ArmSelectXmlRpc);
    }

    else
    {
    	ROS_ERROR("Parameter arm_select not set, shutting down node...");
    	n_.shutdown();
    }

    ArmSelect = (std::string)(ArmSelectXmlRpc);
    if((ArmSelect!="left") && (ArmSelect!="right"))
    {
    	ROS_ERROR("Invalid arm_select parameter, shutting down node... ");
    	n_.shutdown();
    }

    // make sure arm select coincides with joint names in the parameter server
    JointNames.clear();
    JointNames = pc_params_->GetJointNames();
    for(int i=0; i<(int)(JointNames.size()); i++)
    {

    	if((ArmSelect!=(JointNames[i]).substr(0,4))&&(ArmSelect!=(JointNames[i]).substr(0,5)))
    	{
    		ROS_ERROR("arm_select parameter (%s) does not coincide with joint %d name (%s), shutting down node...",
    				ArmSelect.c_str(), i, JointNames[i].c_str());
    		n_.shutdown();
    	}

    }

    pc_params_->SetArmSelect(ArmSelect);


    // initialize joint state messages
    unsigned int dof = pc_params_->GetDOF();
    joint_state_msg_.name = pc_params_->GetJointNames();
    joint_state_msg_.position = std::vector<double>(dof, 0.0);
    joint_state_msg_.velocity = std::vector<double>(dof, 0.0);

    controller_state_msg_.joint_names = pc_params_->GetJointNames();
    controller_state_msg_.actual.positions = std::vector<double>(dof, 0.0);
    controller_state_msg_.actual.velocities = std::vector<double>(dof, 0.0);
    controller_state_msg_.actual.accelerations = std::vector<double>(dof, 0.0);
  }

  /*!
   * \brief Gets parameters from the robot_description and configures the powercube_chain.
   */
  void getRobotDescriptionParameters()
  {
	  unsigned int DOF = pc_params_->GetDOF();
	  std::vector<std::string> JointNames = pc_params_->GetJointNames();

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

	  /// Get max velocities out of urdf model
	  std::vector<double> MaxVelocities(DOF);
	  for (unsigned int i = 0; i < DOF; i++)
	  {
	    MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
	  }
	  ROS_DEBUG("Got max velocities");

	  /// Get lower limits out of urdf model
	  std::vector<double> LowerLimits(DOF);
	  for (unsigned int i = 0; i < DOF; i++)
	  {
		  LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
	  }
	  ROS_DEBUG("Got lower limits");

	  // Get upper limits out of urdf model
	  std::vector<double> UpperLimits(DOF);
	  for (unsigned int i = 0; i < DOF; i++)
	  {
		  UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
	  }
	  ROS_DEBUG("Got upper limits");

      /// Get offsets out of urdf model
	  std::vector<double> Offsets(DOF);
	  for (unsigned int i = 0; i < DOF; i++)
	  {
	  	  Offsets[i] = model.getJoint(JointNames[i].c_str())->calibration->rising.get()[0];
	  }
	  ROS_DEBUG("Got offsets");


	  /// Set parameters
	  pc_params_->SetMaxVel(MaxVelocities);
	  pc_params_->SetLowerLimits(LowerLimits);
	  pc_params_->SetUpperLimits(UpperLimits);
	  pc_params_->SetOffsets(Offsets);
		  
  }

  /*!
   * \brief Executes the callback from the command_pos topic.
   *
   * Set the current position target.
   * \param msg JointPositions
   */
  void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
  {
	  // ROS_WARN("Received new position command. Skipping command: Position commands currently not implemented");
	  ROS_DEBUG("Received new position command");
	  if (initialized_)
	  {
		  PowerCubeCtrl::PC_CTRL_STATUS status;
		  std::vector<std::string> errorMessages;
		  pc_ctrl_->getStatus(status, errorMessages);

		  unsigned int DOF = pc_params_->GetDOF();
		  std::vector<std::string> jointNames = pc_params_->GetJointNames();
		  std::vector<double> cmd_pos(DOF);
		  std::string unit = "rad";

		  /// check dimensions
		  if (msg->positions.size() != DOF)
		  {
			  ROS_ERROR("Skipping command: Commanded positions and DOF are not same dimension.");
			  return;
		  }


		  /// parse positions
		  for (unsigned int i = 0; i < DOF; i++)
		  {
			  /// check joint name
			  if (msg->positions[i].joint_uri != jointNames[i])
			  {
				  ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.",msg->positions[i].joint_uri.c_str(),jointNames[i].c_str(),i);
				  return;
			  }

			  /// check unit
			  if (msg->positions[i].unit != unit)
			  {
				  ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",msg->positions[i].unit.c_str(),unit.c_str());
				  return;
			  }

			  /// if all checks are successful, parse the velocity value for this joint
			  ROS_DEBUG("Parsing position %f for joint %s",msg->positions[i].value,jointNames[i].c_str());
			  cmd_pos[i] = msg->positions[i].value;
		  }

		  /// command positions to powercubes
		  if (!pc_ctrl_->MoveJointSpace(cmd_pos))
		  {
			  ROS_ERROR("Error executing joint space position command in %s arm.", (pc_params_->GetArmSelect()).c_str());
			  error_ = true;
			  error_msg_ = pc_ctrl_->getErrorMessage();
			  ROS_ERROR("Skipping command: %s",pc_ctrl_->getErrorMessage().c_str());//*** have to fix this
			  return;
		  }

		  ROS_DEBUG("Executed position command");

	  }

	  else
	  {
		  ROS_WARN("Skipping position command: powercubes not initialized");
	  }

	  publishState(false);
  }

  /*!
   * \brief Executes the callback from the command_vel topic.
   *
   * Set the current velocity target.
   * \param msg JointVelocities
   */
  void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
  {
	  ROS_DEBUG("Received new velocity command");
	  if (!initialized_)
	  {
		  ROS_WARN("Skipping velocity command: powercubes not initialized");
		  publishState(false);
		  return;
	  }

	  if (pc_ctrl_->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
	  {
		  publishState(false);
		  ROS_ERROR("Error in status of %s arm.", pc_params_->GetArmSelect().c_str());
		  return;
	  }


	  PowerCubeCtrl::PC_CTRL_STATUS status;
	  std::vector<std::string> errorMessages;
	  pc_ctrl_->getStatus(status, errorMessages);

	  /// ToDo: don't rely on position of joint names, but merge them (check between msg.joint_uri and member variable JointStates)

	  unsigned int DOF = pc_params_->GetDOF();
	  std::vector<std::string> jointNames = pc_params_->GetJointNames();
	  std::vector<double> cmd_vel(DOF);
	  std::string unit = "rad";

	  /// check dimensions
	  if (msg->velocities.size() != DOF)
	  {
		  ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
		  return;
	  }

	  /// parse velocities
	  for (unsigned int i = 0; i < DOF; i++)
	  {
		  /// check joint name
		  if (msg->velocities[i].joint_uri != jointNames[i])
		  {
			  ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.",msg->velocities[i].joint_uri.c_str(),jointNames[i].c_str(),i);
			  return;
		  }

		  /// check unit
		  if (msg->velocities[i].unit != unit)
		  {
			  ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",msg->velocities[i].unit.c_str(),unit.c_str());
			  return;
		  }

		  /// if all checks are successful, parse the velocity value for this joint
		  ROS_DEBUG("Parsing velocity %f for joint %s",msg->velocities[i].value,jointNames[i].c_str());
		  cmd_vel[i] = msg->velocities[i].value;
	  }

	  /// command velocities to powercubes
//	  ros::Time t1 = ros::Time::now();
      if (!pc_ctrl_->moveVel(cmd_vel))
	  {
		  error_ = true;
		  error_msg_ = pc_ctrl_->getErrorMessage();
		  ROS_ERROR("Skipping command: %s",pc_ctrl_->getErrorMessage().c_str());
		  return;
	  }
//	  ros::Time t2 = ros::Time::now();
//
//	  ROS_INFO("Move vel command execution time: %d", (t2-t1).toNSec());

	  ROS_DEBUG("Executed velocity command");

	  publishState(false);

  }

  /*!
   * \brief Executes the service callback for init.
   *
   * Connects to the hardware and initialized it.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
	  if (!initialized_)
	  {
		  ROS_INFO("Initializing powercubes...");

		  /// initialize powercubes
          if (pc_ctrl_->init())
		  {

			  initialized_ = true;
			  res.success.data = true;
			  ROS_INFO("...initializing powercubes successful");
              publishState(true);

		  }

		  else
		  {
			  error_ = true;
			  error_msg_ = pc_ctrl_->getErrorMessage();
			  res.success.data = false;
			  res.error_message.data = pc_ctrl_->getErrorMessage();
			  ROS_ERROR("...initializing powercubes not successful. error: %s", res.error_message.data.c_str());
		  }
	  }

	  else
	  {
		  res.success.data = true;
		  res.error_message.data = "powercubes already initialized";
		  ROS_WARN("...initializing powercubes not successful. error: %s",res.error_message.data.c_str());
	  }

	  return true;
  }


  bool srvCallback_Disconnect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
      if (!initialized_)
      {
          ROS_WARN("powercubes already switched off");
          res.success.data = false;
          res.error_message.data = "powercubes already switched off";
      }

      else
      {
          bool closed = pc_ctrl_->Close();
          initialized_ = false;
          res.success.data = true;
          ROS_INFO("Disconnecting %s arm", pc_params_->GetArmSelect().c_str());
      }

      return true;
  }


  /*!
   * \brief Executes the service callback for stop.
   *
   * Stops all hardware movements.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
	  ROS_INFO("Stopping powercubes...");

	  /// stop powercubes
	  if (pc_ctrl_->Stop())
	  {
		  res.success.data = true;
		  ROS_INFO("...stopping powercubes successful.");
	  }

	  else
	  {
		  res.success.data = false;
		  res.error_message.data = pc_ctrl_->getErrorMessage();
		  ROS_ERROR("...stopping powercubes not successful. error: %s", res.error_message.data.c_str());
	  }
	  return true;
  }

  /*!
   * \brief Executes the service callback for recover.
   *
   * Recovers the driver after an emergency stop.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
	  ROS_INFO("Recovering powercubes...");
	  if (initialized_)
	  {
		  /// stopping all arm movements
		  if (pc_ctrl_->Recover())
		  {
			  error_ = false;
			  error_msg_ = "";
			  res.success.data = true;
			  ROS_INFO("...recovering powercubes successful.");
		  }
		  else
		  {
			  res.success.data = false;
			  error_ = true;
			  error_msg_ = pc_ctrl_->getErrorMessage();
			  res.error_message.data = pc_ctrl_->getErrorMessage();
			  ROS_ERROR("...recovering powercubes not successful. error: %s", res.error_message.data.c_str());
		  }
	  }

	  else
	  {
		  res.success.data = false;
		  res.error_message.data = "powercubes not initialized";
		  ROS_ERROR("...recovering powercubes not successful. error: %s",res.error_message.data.c_str());
	  }

	  return true;
  }

   /*!
   * \brief Executes the service callback for SetOperationMode.
   *
   * Sets the driver to different operation modes. Currently only operation_mode=velocity is supported.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
  {
	  // if(req.operation_mode.data != "velocity")
	  // {
	  // 	ROS_WARN("Powercube chain currently only supports velocity commands");
	  // 	res.success.data = false;
	  // }
	  // else
	  // {
	  // 	res.success.data = true;
	  // }
	  ROS_DEBUG("Set Operation Mode Service");

	  if(req.operation_mode.data != "position" && req.operation_mode.data != "velocity")
	  {
		  res.success.data = false;
	  }
	  else
	  {
		  operation_mode_ = req.operation_mode.data;
		  res.success.data = true;
	  }


	  return true;
  }

  /*!
   * \brief Publishes the state of the powercube_chain as ros messages.
   *
   * Published to "/joint_states" as "sensor_msgs/JointState"
   * Published to "state" as "control_msgs/JointTrajectoryState"
   */
  void publishState(bool update=true)
  {
	  ROS_DEBUG("publish state");
	  if (initialized_)
	  {
		  ROS_DEBUG("publish state -- initialized");

		  if(update)
		  {
			  pc_ctrl_->updateStates();
			  ROS_DEBUG("Updated arm");
		  }

          joint_state_msg_.header.stamp = ros::Time::now();
          joint_state_msg_.position = pc_ctrl_->getPositions();
          joint_state_msg_.velocity = pc_ctrl_->getVelocities();


          controller_state_msg_.header.stamp = joint_state_msg_.header.stamp;
          controller_state_msg_.actual.positions = pc_ctrl_->getPositions();
          controller_state_msg_.actual.velocities = pc_ctrl_->getVelocities();
          controller_state_msg_.actual.accelerations = pc_ctrl_->getAccelerations();

		  /// publishing joint and controller states on topic
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
		  joint_state_msg.name = pc_params_->GetJointNames();
		  joint_state_msg.position = std::vector<double>(7,0);
		  joint_state_msg.velocity = std::vector<double>(7,0);

		  control_msgs::JointTrajectoryControllerState controller_state_msg;
		  controller_state_msg.header.stamp = joint_state_msg.header.stamp;
		  controller_state_msg.joint_names = pc_params_->GetJointNames();
		  controller_state_msg.actual.positions = std::vector<double>(7,0);
		  controller_state_msg.actual.velocities = std::vector<double>(7,0);
		  controller_state_msg.actual.accelerations = std::vector<double>(7,0);


//		  ROS_ERROR("PUBLISHING");
		  topicPub_JointState_.publish(joint_state_msg);
          topicPub_ControllerState_.publish(controller_state_msg);

		  last_publish_time_ = joint_state_msg.header.stamp;


	  }

  }


private:

  /// create a handle for this node, initialize node
  ros::NodeHandle n_;

  sensor_msgs::JointState joint_state_msg_;
  control_msgs::JointTrajectoryControllerState controller_state_msg_;

  /// handle for powercube_chain
  boost::scoped_ptr<PowerCubeCtrl> pc_ctrl_;

  /// handle for powercube_chain parameters
  boost::shared_ptr<PowerCubeCtrlParams> pc_params_;




}; //PowerCubeChainNode


/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "powercube_chain");

    ros::NodeHandle nh("~");

    boost::shared_ptr<pthread_mutex_t> CAN_mutex(new pthread_mutex_t);
    *CAN_mutex = PTHREAD_MUTEX_INITIALIZER;
    boost::shared_ptr<canHandle> CAN_handle(new canHandle(0));

	// create PowerCubeChainNode
    PowerCubeChainNode pc_node(nh, CAN_mutex, CAN_handle);
	pc_node.getROSParameters();
	pc_node.getRobotDescriptionParameters();

    /// pointers for parallel gripper node and SDH node
    boost::scoped_ptr<PG70Node> pg70_node;
    boost::scoped_ptr<SdhNode> sdh_node;

    // initialize parallel gripper or SDH node


    // Get which gripper is connected to the arm
    std::string gripper;
    if (nh.hasParam("gripper"))
    {
        nh.getParam("gripper", gripper);
    }

    else
    {
        ROS_WARN("Parameter gripper not set, initializing arm without gripper...");
        gripper = "none";
    }

    ros::NodeHandle gripper_nh(gripper + "_controller");

    if(gripper=="PG70")
    {
        pg70_node.reset(new PG70Node(gripper_nh, CAN_mutex, CAN_handle));
        pg70_node->getROSParameters();
        pg70_node->getRobotDescriptionParameters();
    }

    else if(gripper=="sdh")
    {
        sdh_node.reset(new SdhNode(gripper_nh, CAN_mutex, CAN_handle));

        if(!sdh_node->init())
        {
            ROS_ERROR("Error initializing SDH!!!");
        }
    }

	/// get main loop parameters
	double frequency;
    if (nh.hasParam("frequency"))
	{
        nh.getParam("frequency", frequency);
	}

	else
	{
		//frequency of driver has to be much higher then controller frequency
		frequency = 140 ; //Hz
		ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", frequency);
	}

	ros::Duration min_publish_duration;
    if (nh.hasParam("min_publish_duration"))
	{
		double sec;
        nh.getParam("min_publish_duration", sec);
		min_publish_duration.fromSec(sec);
	}

	else
	{
		ROS_ERROR("Parameter min_publish_time not available");
		return 0;
	}

	if((1.0/min_publish_duration.toSec()) > frequency)
	{
		ROS_ERROR("min_publish_duration has to be longer then delta_t of controller frequency!");
		return 0;
	}


    // read encoder values and publish the joint states before going
    // to the main loop
    for(unsigned int i=0; i<40; i++)
    {
        pc_node.publishState(true);

        if(gripper=="PG70")
        {
            pg70_node->publishState(true);
        }
        else if(gripper=="sdh")
        {
            sdh_node->updateSdh(true);
        }
        ros::Duration(0.05).sleep();
    }



	/// main loop
	ros::Rate loop_rate(frequency); // Hz
    while (nh.ok())
	{


		if ((ros::Time::now() - pc_node.last_publish_time_) >= min_publish_duration)
		{
            pc_node.publishState(false);
		}

        if(gripper=="PG70")
		{
			if((ros::Time::now() - pg70_node->last_publish_time_) >= min_publish_duration)
			{
				pg70_node->publishState(false);
			}
		}

        if(gripper=="sdh")
		{
			if((ros::Time::now() - sdh_node->last_publish_time_) >= min_publish_duration)
			{
				sdh_node->updateSdh(false);
			}
		}

		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
    }

	return 0;
}
