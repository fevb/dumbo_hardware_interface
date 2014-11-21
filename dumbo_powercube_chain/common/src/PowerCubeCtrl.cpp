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
 *   Implementation of powercube control.
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

// ROS includes
#include <ros/ros.h>

// own includes
#include <dumbo_powercube_chain/PowerCubeCtrl.h>
#include <dumbo_powercube_chain/powercube_commands_wrapper.h>
#include <kvaser_canlib/canlib.h>
#include <time.h>


#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}


/*
 * \brief Constructor
 */
PowerCubeCtrl::PowerCubeCtrl(boost::shared_ptr<PowerCubeCtrlParams> params,
                             boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                             boost::shared_ptr<canHandle> CAN_handle)
{

	m_CANDeviceOpened = false;
    m_Initialized = false;

    m_params = params;
    m_CAN_mutex = CAN_mutex;
    m_CAN_handle = CAN_handle;

    // TODO: fix this
    m_horizon = 0.1; // sec

	m_last_time_pub = ros::Time::now();

    m_pc_status = PC_CTRL_OK;

}

/*
 * \brief Destructor
 */
PowerCubeCtrl::~PowerCubeCtrl()
{
	Stop();
	if (m_CANDeviceOpened)
	{
        pthread_mutex_lock(m_CAN_mutex.get());
        PCube_closeDevice(*m_CAN_handle);
        pthread_mutex_unlock(m_CAN_mutex.get());
	}
}

/// ToDo: Check brief
/*!
 * \brief Initializing
 *
 * Setting paramters initialized by PowerCubeCtrlParams.h
 */
bool PowerCubeCtrl::init()
{
	int ret = 0;
	int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	int CanBaudrate = m_params->GetBaudrate();
	std::vector<double> MaxVel = m_params->GetMaxVel();
	std::vector<double> MaxAcc = m_params->GetMaxAcc();
	std::vector<double> Offsets = m_params->GetOffsets();
	std::vector<double> LowerLimits = m_params->GetLowerLimits();
	std::vector<double> UpperLimits = m_params->GetUpperLimits();
	std::vector<std::string> JointNames = m_params->GetJointNames();

    //resize moveVel command variables
    scaled_velocities_.resize(DOF);
    delta_pos_.resize(DOF);
    delta_pos_horizon_.resize(DOF);
    target_pos_.resize(DOF);
    target_pos_horizon_.resize(DOF);
    pos_temp_.resize(DOF);



	/// Output of current settings in the terminal
	std::cout << " D  O  F  :" << DOF << std::endl;
	m_status.resize(DOF);
	m_dios.resize(DOF);
	m_positions.resize(DOF);
	m_velocities.resize(DOF);
    m_accelerations.resize(DOF);

	std::cout << "=========================================================================== " << std::endl;
	std::cout << "PowerCubeCtrl:Init: Trying to initialize with the following parameters: " << std::endl;
	std::cout << "DOF: " << DOF << std::endl;
	std::cout << "CanBaudrate: " << CanBaudrate << std::endl;
	std::cout << "ModulIDs: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << ModulIDs[i] << " ";
	}

	std::cout << std::endl << "maxVel: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << MaxVel[i] << " ";
	}

	std::cout << std::endl << "maxAcc: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << MaxAcc[i] << " ";
	}

	std::cout << std::endl << "upperLimits: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << UpperLimits[i] << " ";
	}

	std::cout << std::endl << "lowerLimits: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << LowerLimits[i] << " ";
	}

	std::cout << std::endl << "offsets: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << Offsets[i] << " ";
	}

	std::cout << std::endl << "=========================================================================== " << std::endl;
	std::ostringstream InitStr;
	InitStr << (m_params->GetArmSelect()).c_str();
	std::cout << "initstring = " << InitStr.str().c_str() << std::endl;

	/// open device
	if(!m_CANDeviceOpened)
	{
		int CAN_Channel;
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_openDevice(m_CAN_handle.get(), &CAN_Channel, InitStr.str());
        pthread_mutex_unlock(m_CAN_mutex.get());
		if (ret != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not open CAN device ";
			m_ErrorMessage = errorMsg.str();
			ROS_ERROR("Could not open CAN device/ arm not found");
			return false;
		}
		m_params->SetCanChannel(CAN_Channel);
	}
	m_CANDeviceOpened = true;
	

	// for (int i = 0; i < DOF; i++)
	//   {
    //     pthread_mutex_lock(m_CAN_mutex.get());
    //     ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
    //     pthread_mutex_unlock(m_CAN_mutex.get());
	//     if(ret<0)
	//       {
	// 	ROS_ERROR("Error resetting module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
	// 	return false;
	//       }
	//   }

	// check status of cubes
	// make sure there is no error
	for(int i = 0; i < DOF; i++)
	{
		unsigned long int state;
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getModuleState(*m_CAN_handle, ModulIDs[i], &state);
        pthread_mutex_unlock(m_CAN_mutex.get());
		if( (ret!=0) || (state & PC_STATE_ERROR))
		{
			ROS_ERROR("State error in module %d , %s arm  after opening device", ModulIDs[i], m_params->GetArmSelect().c_str());
			std::ostringstream errorMsg;
			errorMsg << "Error module state " << i;
			m_ErrorMessage = errorMsg.str();
            pthread_mutex_lock(m_CAN_mutex.get());
            PCube_closeDevice(*m_CAN_handle);
            pthread_mutex_unlock(m_CAN_mutex.get());
			m_CANDeviceOpened = false;
			return false;
		}
	}


	std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
	bool successful = false;
	successful = doHoming();
	if (!successful)
	  {
	    std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
	    return false;
	  }
//
//	ros::Duration(2).sleep();
       


	for (int i = 0; i < DOF; i++)
	  {
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	    if(ret<0)
	      {
		ROS_ERROR("Error resetting module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
		return false;
	      }

	    // max vel
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_setMaxVel(*m_CAN_handle, ModulIDs[i], MaxVel[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	    if(ret<0)
	      {
		ROS_ERROR("Error setting max vel module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
		return false;
	      }

	    // max acceleration
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_setMaxAcc(*m_CAN_handle, ModulIDs[i], MaxAcc[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	    if(ret<0)
	      {
		ROS_ERROR("Error setting max acc module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
		return false;
	      }


	    // Min pos
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_setMinPos(*m_CAN_handle, ModulIDs[i], LowerLimits[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	    if(ret<0)
	      {
		ROS_ERROR("Error setting min pos module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
		return false;
	      }

	    // Max pos
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_setMaxPos(*m_CAN_handle, ModulIDs[i], UpperLimits[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	    if(ret<0)
	      {
		ROS_ERROR("Error setting max pos module %d of %s arm.", ModulIDs[i], (m_params->GetArmSelect().c_str()));
		return false;
	      }

	  }
//
//	ros::Duration(3).sleep();


	// Get joint positions
	for(int i=0; i<DOF; i++)
	{
		float pos;
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getModulePos(*m_CAN_handle, (int)m_params->GetModuleID(i), &pos);
        pthread_mutex_unlock(m_CAN_mutex.get());
		if(ret<0)
		{
			Stop();
			ROS_ERROR("Error getting pos of %d module of %s arm", m_params->GetModuleID(i), m_params->GetArmSelect().c_str());
			return false;
		}
		m_positions[i] = pos;
	}




	/// set synchronous or asynchronous movements
//	if(setSyncMotion())
//	{
//		m_pc_status = PC_CTRL_OK;
//		m_Initialized = true;
//		ROS_INFO("Successfully initialized");
//		return true;
//	}
	//setASyncMotion();
	m_pc_status = PC_CTRL_OK;
	m_Initialized = true;
	ROS_INFO("Successfully initialized");
	return true;

	// All modules initialized successfully
	return false;
}

/*!
 * \brief Close CAN devices
 */
bool PowerCubeCtrl::Close()
{
	if (m_CANDeviceOpened)
	{
		m_Initialized = false;
		m_CANDeviceOpened = false;

        pthread_mutex_lock(m_CAN_mutex.get());
        PCube_closeDevice(*m_CAN_handle);
        pthread_mutex_unlock(m_CAN_mutex.get());

		return true;
	}

	else
	{
		return false;
	}
}

/// ToDo: Check comments
/*!
 * \brief Move joints asynchronous
 *
 *
 */
bool PowerCubeCtrl::MoveJointSpace(const std::vector<double>& target_angles)
{
  PCTRL_CHECK_INITIALIZED();
  std::vector<int> ModuleIDs = m_params->GetModuleIDs();
  int DOF = m_params->GetDOF();
  int ret = 0;
  float pos;

  m_last_time_pub = ros::Time::now();

  for(int i=0; i<DOF; i++)
  {

      pthread_mutex_lock(m_CAN_mutex.get());
      ret += PCube_moveModulePos(*m_CAN_handle, ModuleIDs[i], target_angles[i],
			  &m_status[i], &m_dios[i], &pos);
      pthread_mutex_unlock(m_CAN_mutex.get());
	  m_positions[i] = (double)pos;

  }

  pthread_mutex_lock(m_CAN_mutex.get());
  ret+=PCube_startMotionAll(*m_CAN_handle);
  pthread_mutex_unlock(m_CAN_mutex.get());

  if(ret<0) return false;

  return true;  
}

/// ToDo: Check comments
/*!
 * \brief Move joints synchronous
 *
 * Adjusting velocity of all joints to reach the final angles at the same time
 */
bool PowerCubeCtrl::MoveJointSpaceSync(const std::vector<double>& target)
{
	// PCTRL_CHECK_INITIALIZED();
	// unsigned int DOF = m_params->GetDOF();

	// std::vector<std::string> errorMessages;
	// PC_CTRL_STATUS status;
	// getStatus(status, errorMessages);
	// if ((status != PC_CTRL_OK))
	// {
	// 	m_ErrorMessage.assign("");
	// 	for (unsigned int i = 0; i < DOF; i++)
	// 	{
	// 		m_ErrorMessage.append(errorMessages[i]);
	// 	}
	// 	return false;
	// }

	// std::vector<double> vel(DOF);
	// std::vector<double> acc(DOF);

	// double TG = 0;

	// try
	// {
	// 	/// calculate which joint takes the longest time to reach goal
	// 	std::vector<double> times(DOF);
	// 	for (unsigned int i = 0; i < DOF; i++)
	// 	{
	// 		RampCommand rm(m_positions[i], m_velocities[i], target[i], m_params->GetMaxAcc()[i],
	// 					   m_params->GetMaxVel()[i]);
	// 		times[i] = rm.getTotalTime();
	// 	}

	// 	/// determine the joint index that has the greatest value for time
	// 	int furthest = 0;
	// 	double max = times[0];
	// 	for (unsigned int i = 1; i < DOF; i++)
	// 	{
	// 		if (times[i] > max)
	// 		{
	// 			max = times[i];
	// 			furthest = i;
	// 		}
	// 	}

	// 	RampCommand rm_furthest(m_positions[furthest], m_velocities[furthest], target[furthest],
        //                         m_params->GetMaxAcc()[furthest], m_params->GetMaxVel()[furthest]);

	// 	double T1 = rm_furthest.T1();
	// 	double T2 = rm_furthest.T2();
	// 	double T3 = rm_furthest.T3();

	// 	/// total time:
	// 	TG = T1 + T2 + T3;

	// 	/// calculate velocity and acceleration for all joints:
	// 	acc[furthest] = m_params->GetMaxAcc()[furthest];
	// 	vel[furthest] = m_params->GetMaxVel()[furthest];
	// 	for (unsigned int i = 0; i < DOF; i++)
	// 	{
	// 		if (int(i) != furthest)
	// 		{
	// 			double a;
	// 			double v;
	// 			RampCommand::calculateAV(m_positions[i], m_velocities[i], target[i], TG, T3, m_params->GetMaxAcc()[i],
	// 									 m_params->GetMaxVel()[i], a, v);

	// 			acc[i] = a;
	// 			vel[i] = v;
	// 		}
	// 	}
	// }
	// catch (...)
	// {
	// 	return false;
	// }

	// /// Send motion commands to hardware
	// for (unsigned int i = 0; i < DOF; i++)
	// {
    // 	pthread_mutex_lock(m_CAN_mutex.get());
    // 	PCube_moveRamp(*m_CAN_handle, m_params->GetModuleIDs()[i], target[i], fabs(vel[i]), fabs(acc[i]));
    // 	pthread_mutex_unlock(m_CAN_mutex.get());
	// }

    // pthread_mutex_lock(m_CAN_mutex.get());
    // PCube_startMotionAll(*m_CAN_handle);
    // pthread_mutex_unlock(m_CAN_mutex.get());

	return true;
}

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */
bool PowerCubeCtrl::moveVel(const std::vector<double>& vel)
{
	PCTRL_CHECK_INITIALIZED();

	//== init var ==================================================

	/// getting paramerters
    unsigned int DOF = m_params->GetDOF();

	float target_time; 	// time in milliseconds
	float target_time_horizon;

	float delta_t;			// time from the last moveVel cmd to now

	/// getting limits
    const std::vector<double> &LowerLimits = m_params->GetLowerLimits();
    const std::vector<double> &UpperLimits = m_params->GetUpperLimits();
    const std::vector<double> &maxVels = m_params->GetMaxVel();

    const std::vector<double> &maxAcc = m_params->GetMaxAcc();

	int ret; 		// temp return value holder
	float pos; 	// temp position variable for PCube_move.. cmds


	struct timespec sleep_time;

	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 1000000; // 1 ms

	/// check dimensions
    if (vel.size() != DOF)
	{
		m_ErrorMessage = "Skipping command: Commanded velocities and DOF are not same dimension.";
		return false;
	}


	// scale down velocities to comply with max velocity limits
	double scale = 1.0;
	double scale_temp;
	for(unsigned int i=0; i<DOF; i++)
	{
        if(vel[i] > maxVels[i])
		{
            scale_temp  = (double)vel[i]/(double)maxVels[i];
			if((scale_temp > 0.0) && (scale_temp > scale)) scale = scale_temp;
		}

        else if(vel[i]*-1.0 > maxVels[i])
		{
            scale_temp = -1.0*((double)vel[i]/(double)maxVels[i]);
			if((scale_temp > 0.0) && (scale_temp > scale)) scale = scale_temp;
		}
	}


	for(unsigned int i=0; (i<DOF) && (scale>1); i++)
	{
        scaled_velocities_[i] = (double)vel[i]/(double)scale;
	}

	//== calculate destination position ============================
	// needed for limit handling and MoveStep command

	delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
	m_last_time_pub = ros::Time::now();

	//check for acceleration limits and scale
	//todo

	bool acc_limit_passed = false;
	for(unsigned int i=0; i < DOF; i++)
	{
        if(fabs(scaled_velocities_[i] - m_velocities[i]) > fabs(maxAcc[i]))
			acc_limit_passed = true;
	}

	if(acc_limit_passed)
	{
		for(unsigned int i=0; i < DOF; i++)
		{
		  ROS_ERROR("Acceleration limit surpassed, sending zero velocity to the %s arm!", m_params->GetArmSelect().c_str());
          scaled_velocities_[i] = 0.0;
		}
	}

	// calculate target position
	for (unsigned int i = 0; i < DOF; i++)
	{
		// limit step time to 50msec
		//TODO: set value 0.05 as parameter
		if (delta_t >= 0.050)
		{
			target_time = 0.050; //sec
		}
		else
		{
			target_time = delta_t;
		}

		//add horizon to time before calculation of target position, to influence both time and position at the same time
		target_time_horizon = target_time + (float)m_horizon; //sec


        delta_pos_horizon_[i] = target_time_horizon * scaled_velocities_[i];
        delta_pos_[i] = target_time * scaled_velocities_[i];

        ROS_DEBUG("delta_pos[%i]: %f target_time: %f velocity[%i]: %f",i ,delta_pos_[i], target_time, i, scaled_velocities_[i]);

		// calculate target position
        target_pos_horizon_[i] = m_positions[i] + delta_pos_horizon_[i];
        target_pos_[i] = m_positions[i] + delta_pos_[i];
        ROS_DEBUG("target_pos[%i]: %f m_position[%i]: %f",i ,target_pos_[i], i, m_positions[i]);
	}


	// check joint position limits
	for (unsigned int i = 0; i < DOF; i++)
	{



		/// check position limits
		// TODO: add second limit "safty limit"
		// if target position is outer limits and the command velocity is in in direction away from working range, skip command
        if ((target_pos_horizon_[i] < LowerLimits[i]) && (scaled_velocities_[i] < 0))
		{
            //ROS_INFO("Skipping command: %f Target position exceeds lower limit (%f) for joint %d.", target_pos_horizon[i], LowerLimits[i], i+1);
			return true;
		}

		// if target position is outer limits and the command velocity is in in direction away from working range, skip command
        if ((target_pos_horizon_[i] > UpperLimits[i]) && (scaled_velocities_[i] > 0))
		{
            //ROS_INFO("Skipping command: %f Target position exceeds upper limit (%f) for joint %d.", target_pos_horizon[i], UpperLimits[i], i+1);

			return true;
		}
	}



	//== send velocity cmd to modules ==============================

	//convert the time to int in [ms]
	unsigned short time4motion = (unsigned short)((target_time_horizon)*1000.0);
    pthread_mutex_lock(m_CAN_mutex.get());
	for (unsigned int i = 0; i < DOF; i++)
	{
		// in order to not overload the CAN bus, we sleep between sends:
//		nanosleep(&sleep_time,NULL);


//		pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_moveStepExtended(*m_CAN_handle, m_params->GetModuleID(i), target_pos_horizon_[i], time4motion, &m_status[i], &m_dios[i], &pos);
//		pthread_mutex_unlock(m_CAN_mutex.get());

		/// error handling
		if (ret != 0)
		{
            ROS_ERROR("Com Error");
			pos = m_positions[i];
			//m_pc_status = PC_CTRL_ERR;
			//TODO: add error msg for diagnostics
		}

		// !!! Position in pos is position before moveStep movement, to get the expected position after the movement (required as input to the next moveStep command) we add the delta position (cmd_pos) !!!
        m_positions[i] = (double)pos;
        pos_temp_[i] = (double)pos;
	}

    updateVelocities(pos_temp_, delta_t);

    pthread_mutex_unlock(m_CAN_mutex.get());
	return true;
}

// Calculation of velocities based on vel = 1/(6*dt) * (-pos(t-3) - 3*pos(t-2) + 3*pos(t-1) + pos(t))
void PowerCubeCtrl::updateVelocities(std::vector<double> pos_temp, double delta_t)
{
	unsigned int DOF = m_params->GetDOF();
	if(m_cached_pos.size() < 4)
	{
		m_cached_pos.push_back(pos_temp);

		for(unsigned int i = 0; i < DOF; i++)
		{
			m_velocities[i] = 0.0;
		}
	}
	else
	{
		m_cached_pos.push_back(pos_temp);
		m_cached_pos.pop_front();
		for(unsigned int i = 0; i < DOF; i++)
		{
			m_velocities[i] = 1/(6*delta_t) * (-m_cached_pos[0][i]-(3*m_cached_pos[1][i])+(3*m_cached_pos[2][i])+m_cached_pos[3][i]);
		}
	}
}

/*!
 * \brief Stops the manipulator immediately
 */
bool PowerCubeCtrl::Stop()
{
	/// stop should be executed without checking any conditions
    pthread_mutex_lock(m_CAN_mutex.get());
    PCube_haltAll(*m_CAN_handle);
    pthread_mutex_unlock(m_CAN_mutex.get());

	/// after halt the modules don't accept move commands any more, they first have to be reset
    ros::Duration(0.5).sleep();

	return true;
}

/*!
 * \brief Recovers the manipulator after an emergency stop
 */
bool PowerCubeCtrl::Recover()
{
	 unsigned int DOF = m_params->GetDOF();

	  std::vector<std::string> errorMessages;
	  PC_CTRL_STATUS status;

	  unsigned long state = PC_CTRL_OK;
	  unsigned char dio;
	  float position;
	  int ret = 0;

	  // check for each module if reset is necessary
	  for (unsigned int i = 0; i < DOF; i++)
	  {

          pthread_mutex_lock(m_CAN_mutex.get());
          ret = PCube_resetModule(*m_CAN_handle, m_params->GetModuleID(i));
          pthread_mutex_unlock(m_CAN_mutex.get());
	  }

	  // time for reboot
	  usleep(500000);

	  // check is everything is ok now
	  updateStates();

	  if (m_pc_status == PC_CTRL_NOT_HOMED)
	  {
	    if (!doHoming())
			{
			  return false;
			}
	  }

	  usleep(500000);

	  // modules should be recovered now
	  m_pc_status = PC_CTRL_OK;

	  updateStates();
	  // check if modules are really back to normal state
	  getStatus(status, errorMessages);

	  if ((status != PC_CTRL_OK))
	  {
		  m_ErrorMessage.assign("");

	    for (int i = 0; i < m_params->GetDOF(); i++)
			{
			  m_ErrorMessage.append(errorMessages[i]);
			}
	    return false;
	  }

	  /// modules successfully recovered
//	  m_Initialized = true;
	  m_pc_status = PC_CTRL_OK;
	return true;
}

/*!
 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
 *
 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
 */
bool PowerCubeCtrl::setMaxVelocity(double maxVelocity)
{
	PCTRL_CHECK_INITIALIZED();
	for (int i = 0; i < m_params->GetDOF(); i++)
	{
        pthread_mutex_lock(m_CAN_mutex.get());
        PCube_setMaxVel(*m_CAN_handle, m_params->GetModuleID(i), maxVelocity);
        pthread_mutex_unlock(m_CAN_mutex.get());

		std::vector<double> maxVelocities(maxVelocity);
		m_params->SetMaxVel(maxVelocities);
	}


	return true;
}

/*!
 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
 */
bool PowerCubeCtrl::setMaxVelocity(const std::vector<double>& maxVelocities)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_params->GetDOF(); i++)
    {
      pthread_mutex_lock(m_CAN_mutex.get());
      PCube_setMaxVel(*m_CAN_handle, m_params->GetModuleID(i), maxVelocities[i]);
      pthread_mutex_unlock(m_CAN_mutex.get());
    }
	
  m_params->SetMaxVel(maxVelocities);

  return true;
}

/*!
 * \brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
 *
 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
 */
bool PowerCubeCtrl::setMaxAcceleration(double maxAcceleration)
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
        pthread_mutex_lock(m_CAN_mutex.get());
        PCube_setMaxAcc(*m_CAN_handle, m_params->GetModuleID(i), maxAcceleration);
        pthread_mutex_unlock(m_CAN_mutex.get());
		std::vector<double> maxAccelerations(maxAcceleration);
		m_params->SetMaxAcc(maxAccelerations);
	}

	return true;
}

/*!
 * \brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
 */
bool PowerCubeCtrl::setMaxAcceleration(const std::vector<double>& maxAccelerations)
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
        pthread_mutex_lock(m_CAN_mutex.get());
        PCube_setMaxAcc(*m_CAN_handle, m_params->GetModuleID(i), maxAccelerations[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
	}
	m_params->SetMaxAcc(maxAccelerations);
	return true;
}

/*!
 * \brief Sets the horizon (sec).
 *
 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
 * of a failure this is the time the powercube chain will continue to move until it is stopped.
 */
bool PowerCubeCtrl::setHorizon(double horizon)
{
	m_horizon = horizon;

	return true;
}

/*!
 * \brief Gets the horizon (sec).
 *
 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
 * of a failure this is the time the powercube chain will continue to move until it is stopped.
 */
double PowerCubeCtrl::getHorizon()
{
	return m_horizon;
}

/*!
 * \brief Configure powercubes to start all movements synchronously
 *
 * Tells the Modules not to start moving until PCube_startMotionAll is called.
 */
bool PowerCubeCtrl::setSyncMotion()
{
	if (m_CANDeviceOpened)
	{
		for (int i = 0; i < m_params->GetDOF(); i++)
		{
			unsigned long int confword;

			/// get config
            pthread_mutex_lock(m_CAN_mutex.get());
            PCube_getConfig(*m_CAN_handle, m_params->GetModuleID(i), &confword);
            pthread_mutex_unlock(m_CAN_mutex.get());

			/// set config to synchronous
            pthread_mutex_lock(m_CAN_mutex.get());
            PCube_setConfig(*m_CAN_handle, m_params->GetModuleID(i), confword | PC_CONFIGID_MOD_SYNC_MOTION);
            pthread_mutex_unlock(m_CAN_mutex.get());
		}
		return true;
	}
	else
	{
		return false;
	}

  return true;
}
/*!
 * \brief Configure powercubes to start all movements asynchronously
 *
 * Tells the Modules to start immediately
 */
bool PowerCubeCtrl::setASyncMotion()
{
	if (m_CANDeviceOpened)
	{
		for (int i = 0; i < m_params->GetDOF(); i++)
		{
			unsigned long int confword;

			/// get config
            pthread_mutex_lock(m_CAN_mutex.get());
            PCube_getConfig(*m_CAN_handle, m_params->GetModuleID(i), &confword);
            pthread_mutex_unlock(m_CAN_mutex.get());

			/// set config to asynchronous
            pthread_mutex_lock(m_CAN_mutex.get());
            PCube_setConfig(*m_CAN_handle, m_params->GetModuleID(i), confword & (~PC_CONFIGID_MOD_SYNC_MOTION));
            pthread_mutex_unlock(m_CAN_mutex.get());
		}
		return true;
	}

	else
	{
		return false;
	}
  return true;
}
/*!
 * \brief Returns the current states
 */
bool PowerCubeCtrl::updateStates()
{
	PCTRL_CHECK_INITIALIZED();
	unsigned int DOF = m_params->GetDOF();
	unsigned long state;
	PC_CTRL_STATUS pc_status = PC_CTRL_ERR;
	std::vector<std::string> ErrorMessages;
	std::ostringstream errorMsg;

	unsigned char dio;
	float position;
	int ret = 0;

	for (unsigned int i = 0; i < DOF; i++)
	{
		ros::Duration(0.001).sleep();
		state = m_status[i];
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getStateDioPos(*m_CAN_handle, m_params->GetModuleID(i), &state, &dio, &position);
        pthread_mutex_unlock(m_CAN_mutex.get());

		if (ret != 0)
		{
			//m_pc_status = PC_CTRL_ERR;
			ROS_ERROR("Error on com in UpdateStates");
			return true;
			//errorMsg << "State: Error com with Module [" <<  i << "]";
			//ErrorMessages[i] = errorMsg.str();
		}
		else
		{
			ROS_DEBUG("Module %i, State: %li, Time: %f",i, state, ros::Time::now().toSec());

			m_status[i] = state;
			m_dios[i] = dio;
			m_positions[i] = (double)position;
		}

	}


	double delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
	m_last_time_pub = ros::Time::now();

	updateVelocities(m_positions, delta_t);

	// evaluate state for translation for diagnostics msgs
	getStatus(pc_status, ErrorMessages);

	for (unsigned int i=0; i<ErrorMessages.size(); i++)
	{
		m_ErrorMessage.clear();
		m_ErrorMessage.assign("");
		m_ErrorMessage.append(ErrorMessages[i]);
	}


	return true;
}

/*!
 * \brief Gets the status of the modules
 */
bool PowerCubeCtrl::getStatus(PC_CTRL_STATUS& status, std::vector<std::string>& errorMessages)
{
	unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModuleIDs = m_params->GetModuleIDs();

	errorMessages.clear();
	errorMessages.resize(DOF);

	status = PC_CTRL_ERR;

	for (unsigned int i = 0; i < DOF; i++)
	{
		std::ostringstream errorMsg;

		if (m_status[i] & STATEID_MOD_POW_VOLT_ERR)
		{
			errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
			errorMsg << "Motor voltage below minimum value!";
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_POW_VOLT_ERR;
		}
		else if (!(m_status[i] & STATEID_MOD_HOME))
		{
			errorMsg << "Warning: Module " << ModuleIDs[i];
			errorMsg << " is not referenced!";
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_NOT_HOMED;
		}

		else if (m_status[i] & STATEID_MOD_ERROR)
		{
			errorMsg << "Error in  Module " << ModuleIDs[i];
			errorMsg << " : Status code: " << std::hex << m_status[i];
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_ERR;
		}
		else if (m_pc_status & PC_CTRL_ERR)
		{
			errorMsg << "PowerCubeCtrl is in global error state";
			errorMessages[i] = errorMsg.str();
			status =  PC_CTRL_ERR;
		}
		else
		{
			errorMsg << "Module with Id " << ModuleIDs[i];
			errorMsg << ": Status OK.";
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_OK;
		}
	}

	m_pc_status = status;

	return true;
}

/*!
 * \brief Returns true if some cubes are still moving
 */
bool PowerCubeCtrl::statusMoving()
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		if (m_status[i] & STATEID_MOD_MOTION)
			return true;
	}
	return false;
}

/*!
 * \brief Gets the current positions
 */
const std::vector<double> &PowerCubeCtrl::getPositions()
{
	return m_positions;
}

/*!
 * \brief Gets the current velocities
 */
const std::vector<double> &PowerCubeCtrl::getVelocities()
{
	/// ToDo: calculate new velocities before returning
	return m_velocities;
}

/*!
 * \brief Gets the current positions
 */
const std::vector<double> &PowerCubeCtrl::getAccelerations()
{
	/// ToDo: calculate new accelerations before returning
	return m_accelerations;
}

/*!
 * \brief Does homing for all Modules
 */
bool PowerCubeCtrl::doHoming()
{
  unsigned int DOF = m_params->GetDOF();
  std::vector<int> ModuleIDs = m_params->GetModuleIDs();
  bool homed = 0;
  int counter = 0;
  unsigned long int state;

  /// start homing
  int ret = 0;
  for (unsigned int i = 0; i < DOF; i++)
  {
      pthread_mutex_lock(m_CAN_mutex.get());
      ret = PCube_homeModule(*m_CAN_handle, ModuleIDs[i]);
      pthread_mutex_unlock(m_CAN_mutex.get());
	  if (ret != 0)
	  {
		  ROS_ERROR("Can't home module %d", ModuleIDs[i]);
		  std::ostringstream errorMsg;
		  errorMsg << "Can't start homing for module " << ModuleIDs[i];
		  m_ErrorMessage = errorMsg.str();
		  m_pc_status = PC_CTRL_NOT_HOMED;
		  return false;
	  }
  }

  /// wait until all modules are homed

  while(!homed && counter++ < 200)
  {
	  ros::Duration(0.5).sleep();
	  homed = true;
	  for(unsigned int i=0; i<1; i++)
	  {
          pthread_mutex_lock(m_CAN_mutex.get());
          (void)PCube_getModuleState(*m_CAN_handle, ModuleIDs[i], &state);
          pthread_mutex_unlock(m_CAN_mutex.get());
		  if(state & PC_STATE_ERROR)
		  {
		    ROS_ERROR("State error module %d, %s arm after homing", ModuleIDs[i], m_params->GetArmSelect().c_str());
			  m_pc_status = PC_CTRL_NOT_HOMED;
			  return false;
		  }

		  if(!(state & PC_STATE_HOME_OK))
			  homed = false;

	  }
  }

  if((!homed) || (counter>=200))
    {
      m_pc_status = PC_CTRL_NOT_HOMED;
      return false;
    }

  // modules successfully homed
  m_pc_status = PC_CTRL_OK;
  ROS_INFO("%s arm successfully homed.", (m_params->GetArmSelect()).c_str());
  return true;
}
