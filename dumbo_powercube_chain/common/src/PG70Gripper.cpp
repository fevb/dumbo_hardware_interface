/*
 *  PG70Gripper.cpp
 *
 *  For controlling the parallel PG70 gripper
 *  Created on: Aug 10, 2012
 *  Authors:   Francisco Viña 
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Vina, CVAP, KTH
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


#include <ros/ros.h>
#include <dumbo_powercube_chain/PG70Gripper.h>
#include <dumbo_powercube_chain/powercube_commands_wrapper.h>
#include <pthread.h>

#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

PG70Gripper::PG70Gripper(PowerCubeCtrlParams * params) :
PowerCubeCtrl(params)
{

	m_CANDeviceOpened = false;
	m_Initialized = false;
	m_motion = false;
	m_params = params;
	m_last_time_pub = ros::Time::now();

	m_pc_status = PC_CTRL_OK;
}


PG70Gripper::~PG70Gripper() {
	// LWA powercube chain class takes care of closing the CAN bus
	CloseDevice();
}

bool PG70Gripper::Init()
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

	m_status.resize(DOF);
	m_dios.resize(DOF);
	m_positions.resize(DOF);
	m_velocities.resize(DOF);

	unsigned long int SerialNumber;

	if(DOF==0)
	{
		ROS_ERROR("Error on gripper: zero DOF.");
		return false;
	}

	std::cout << "Initializing PG70 gripper on the " << m_params->GetArmSelect().c_str();
	std:: cout << " arm, module id: " << m_params->GetModuleID(0) << " handle: " << PowerCubeCtrl::m_DeviceHandle<< std::endl;

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
	std::cout << std::endl;

	for(int i=0; i<DOF; i++)
	{

		pthread_mutex_lock(&m_mutex);
		ret = PCube_getSerialNumber(m_DeviceHandle, m_params->GetModuleID(i), &SerialNumber);
		pthread_mutex_unlock(&m_mutex);
		if(ret!=0) return false;

		if((int)SerialNumber == (int)m_params->GetSerialNumber())
		{

//			// home
//			pthread_mutex_lock(&m_mutex);
//			ret = PCube_homeModule(m_DeviceHandle, ModulIDs[i]);
//			pthread_mutex_unlock(&m_mutex);
//			if(ret<0)
//			{
//				ROS_ERROR("Error homing gripper of %s arm.", (m_params->GetArmSelect().c_str()));
//				return false;
//			}
//
//
//			// wait until gripper is homed
//			unsigned long int state = 0;
//			while(!(state & PC_STATE_HOME_OK))
//			{
//				ros::Duration(0.5).sleep();
//				pthread_mutex_lock(&m_mutex);
//				ret = PCube_getModuleState(m_DeviceHandle, ModulIDs[i], &state);
//				pthread_mutex_unlock(&m_mutex);
//			}

			// reset
			pthread_mutex_lock(&m_mutex);
			ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}


			// max vel
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxVel(m_DeviceHandle, ModulIDs[i], MaxVel[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max vel of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			// max acceleration
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxAcc(m_DeviceHandle, ModulIDs[i], MaxAcc[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max acc of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}


			// Min pos
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMinPos(m_DeviceHandle, ModulIDs[i], (LowerLimits[i])*2.0);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting min pos of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			// Max pos
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxPos(m_DeviceHandle, ModulIDs[i], (UpperLimits[i])*2.0);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max pos of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}


			// reset
			pthread_mutex_lock(&m_mutex);
			ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			//get gripper pos
			float gripper_pos;
			pthread_mutex_lock(&m_mutex);
			ret = PCube_getModulePos(m_DeviceHandle, ModulIDs[i], &gripper_pos);
			pthread_mutex_unlock(&m_mutex);

			if(ret<0)
			{
				ROS_ERROR("Error getting gripper pos of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}
			m_positions[i] = (((double)gripper_pos)/2.0);

			//get gripper Max current
			float MaxCurrent;
			pthread_mutex_lock(&m_mutex);
			ret = PCube_getMaxCurrent(m_DeviceHandle, ModulIDs[i], &MaxCurrent);
			pthread_mutex_unlock(&m_mutex);

			if(ret<0)
			{
				ROS_ERROR("Error getting max current of gripper.");
				return false;
			}

			else
			{
				setMaxCurrent((double)MaxCurrent);
			}

		}

		else
		{
			ROS_ERROR("Error gripper: Serial numbers don't coincide... %d %d", (int)SerialNumber, (int)m_params->GetSerialNumber());
			return false;
		}
	}

	m_pc_status = PC_CTRL_OK;
	m_Initialized = true;
	m_CANDeviceOpened = true;
	ROS_INFO("Successfully initialized PG70 gripper on %s arm", m_params->GetArmSelect().c_str());

	return true;
}

bool PG70Gripper::Recover()
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	std::vector<double> MaxVel = m_params->GetMaxVel();
	std::vector<double> MaxAcc = m_params->GetMaxAcc();
	int DOF = m_params->GetDOF();

	for(int i=0; i<DOF; i++)
	{

		// reset
		pthread_mutex_lock(&m_mutex);
		ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
		pthread_mutex_unlock(&m_mutex);
		if(ret<0)
		{
			ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
			return false;
		}

//		// home
//		pthread_mutex_lock(&m_mutex);
//		ret = PCube_homeModule(m_DeviceHandle, ModulIDs[i]);
//		pthread_mutex_unlock(&m_mutex);
//		if(ret<0)
//		{
//			ROS_ERROR("Error homing gripper of %s arm.", (m_params->GetArmSelect().c_str()));
//			return false;
//		}
//
//
//		// wait until gripper is homed
//		unsigned long int state = 0;
//		while(!(state & PC_STATE_HOME_OK))
//		{
//			ros::Duration(0.5).sleep();
//			pthread_mutex_lock(&m_mutex);
//			ret = PCube_getModuleState(m_DeviceHandle, ModulIDs[i], &state);
//			pthread_mutex_unlock(&m_mutex);
//		}
//
//		// reset
//		pthread_mutex_lock(&m_mutex);
//		ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
//		pthread_mutex_unlock(&m_mutex);
//		if(ret<0)
//		{
//			ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
//			return false;
//		}
//
//
//		// max vel
//		if((target_vel<MaxVel[i]) && (target_vel>0.0))
//		{
//			pthread_mutex_lock(&m_mutex);
//			ret = PCube_setMaxVel(m_DeviceHandle, ModulIDs[i], target_vel);
//			pthread_mutex_unlock(&m_mutex);
//			if(ret<0)
//			{
//				ROS_ERROR("Error setting max vel of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
//				return false;
//			}
//		}
//
//		else
//		{
//			return false;
//		}
//
//		// max acceleration
//		pthread_mutex_lock(&m_mutex);
//		ret = PCube_setMaxAcc(m_DeviceHandle, ModulIDs[i], MaxAcc[i]);
//		pthread_mutex_unlock(&m_mutex);
//		if(ret<0)
//		{
//			ROS_ERROR("Error setting max acc of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
//			return false;
//		}

	}

	//*** have to do...
	return true;
}

bool PG70Gripper::updateStates()
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
	int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	unsigned char dio;
	unsigned long int state;

	//get gripper pos
	for(int i=0; i<DOF; i++)
	{
		float gripper_pos;
		pthread_mutex_lock(&m_mutex);
		ret = PCube_getStateDioPos(m_DeviceHandle, ModulIDs[i], &state, &dio, &gripper_pos);
		pthread_mutex_unlock(&m_mutex);

		if(ret<0)
		{
			ROS_ERROR("Error getting gripper pos of %s arm.", (m_params->GetArmSelect().c_str()));
			return false;
		}
		m_positions[i] = (((double)gripper_pos)/2.0);
		m_status[i] = state;
		m_dios[i] = dio;
	}

	return true;
}

bool PG70Gripper::MovePos(double target_pos)
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
	float gripper_pos;
	double UpperLimit = (m_params->GetUpperLimits().at(0))*2.0;
	double LowerLimit = (m_params->GetLowerLimits().at(0))*2.0;
	double target_pos_conv = target_pos*2.0; // converts target pos
	unsigned long int status;

	if(!DoHoming())
	{
		return false;
	}
	if((target_pos_conv < UpperLimit) &&  (target_pos_conv > LowerLimit))
	{

		pthread_mutex_lock(&m_mutex);
		ret = PCube_moveModulePos(m_DeviceHandle, m_params->GetModuleID(0), (float)target_pos_conv,
				&status, &m_dios[0], &gripper_pos);
		pthread_mutex_unlock(&m_mutex);
		m_positions[0] = ((double)gripper_pos)/2.0;
	}

	else
	{
		ROS_ERROR("Gripper position outside limits: %f [mm]", target_pos_conv*1000.0);
		return false;
	}

	if(ret<0)
	{
		return false;
	}
	m_motion = true;
	return true;
}

bool PG70Gripper::CloseGripper(double target_vel, double current_limit)
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
	double MaxVel = m_params->GetMaxVel().at(0);

	if(!DoHoming())
	{
		return false;
	}

	for(int i=0; i<m_params->GetDOF(); i++)
	{
		if((current_limit < getMaxCurrent()) && (current_limit > 0))
		{
			pthread_mutex_lock(&m_mutex);
			ret += PCube_setMaxCurrent(m_DeviceHandle, m_params->GetModuleID(i), (float)current_limit);
			pthread_mutex_unlock(&m_mutex);
		}

		else
		{
			ROS_ERROR("Exceeded max current limit of PG70 gripper");
			return false;
		}

		if((target_vel < MaxVel) && (target_vel > 0.0))
		{
			pthread_mutex_lock(&m_mutex);
			ret += PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), (float)target_vel);
			pthread_mutex_unlock(&m_mutex);
		}
		else
		{
			ROS_ERROR("Exceeded max vel limit of PG70 gripper");
			return false;
		}

	}
	if(ret<0) return false;

	return MovePos(0.001);
}

bool PG70Gripper::DoHoming()
{
	int ret;
	int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	std::vector<double> MaxVel = m_params->GetMaxVel();
	std::vector<double> MaxAcc = m_params->GetMaxAcc();
	std::vector<double> Offsets = m_params->GetOffsets();
	std::vector<double> LowerLimits = m_params->GetLowerLimits();
	std::vector<double> UpperLimits = m_params->GetUpperLimits();

	PCTRL_CHECK_INITIALIZED();
	for(unsigned int i=0; i<DOF; i++)
	{
		unsigned long int state = 0;
		pthread_mutex_lock(&m_mutex);
		ret = PCube_getModuleState(m_DeviceHandle, ModulIDs[i], &state);
		pthread_mutex_unlock(&m_mutex);

		if(!(state & PC_STATE_HOME_OK))
		{

			// home
			pthread_mutex_lock(&m_mutex);
			ret = PCube_homeModule(m_DeviceHandle, ModulIDs[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error homing gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			// wait until gripper is homed
			state = 0;
			while(!(state & PC_STATE_HOME_OK))
			{
				ros::Duration(0.5).sleep();
				pthread_mutex_lock(&m_mutex);
				ret = PCube_getModuleState(m_DeviceHandle, ModulIDs[i], &state);
				pthread_mutex_unlock(&m_mutex);
			}

			// reset
			pthread_mutex_lock(&m_mutex);
			ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}



			// max vel
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxVel(m_DeviceHandle, ModulIDs[i], MaxVel[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max vel of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			// max acceleration
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxAcc(m_DeviceHandle, ModulIDs[i], MaxAcc[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max acc of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}


			// Min pos
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMinPos(m_DeviceHandle, ModulIDs[i], (LowerLimits[i])*2.0);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting min pos of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			// Max pos
			pthread_mutex_lock(&m_mutex);
			ret = PCube_setMaxPos(m_DeviceHandle, ModulIDs[i], (UpperLimits[i])*2.0);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error setting max pos of gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}


			// reset
			pthread_mutex_lock(&m_mutex);
			ret = PCube_resetModule(m_DeviceHandle, ModulIDs[i]);
			pthread_mutex_unlock(&m_mutex);
			if(ret<0)
			{
				ROS_ERROR("Error resetting gripper of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}

			//get gripper pos
			float gripper_pos;
			pthread_mutex_lock(&m_mutex);
			ret = PCube_getModulePos(m_DeviceHandle, ModulIDs[i], &gripper_pos);
			pthread_mutex_unlock(&m_mutex);

			if(ret<0)
			{
				ROS_ERROR("Error getting gripper pos of %s arm.", (m_params->GetArmSelect().c_str()));
				return false;
			}
			m_positions[i] = (((double)gripper_pos)/2.0);

			//get gripper Max current
			float MaxCurrent;
			pthread_mutex_lock(&m_mutex);
			ret = PCube_getMaxCurrent(m_DeviceHandle, ModulIDs[i], &MaxCurrent);
			pthread_mutex_unlock(&m_mutex);

			if(ret<0)
			{
				ROS_ERROR("Error getting max current of gripper.");
				return false;
			}

			else
			{
				setMaxCurrent((double)MaxCurrent);
			}
		}
	}

	return true;
}

bool PG70Gripper::inMotion()
{
	if(m_motion)
	{
		m_motion = false;
		return true;
	}

	return (m_status[0] & PC_STATE_MOTION);
}

