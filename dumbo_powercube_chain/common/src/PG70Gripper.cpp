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

PG70Gripper::PG70Gripper(boost::shared_ptr<PowerCubeCtrlParams> params,
                         boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                         boost::shared_ptr<canHandle> CAN_handle) :
    PowerCubeCtrl(params, CAN_mutex, CAN_handle),
    m_executing_pos_command(false)
{

}

PG70Gripper::PG70Gripper(boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                         boost::shared_ptr<canHandle> CAN_handle) :
    PowerCubeCtrl(CAN_mutex, CAN_handle),
    m_executing_pos_command(false)
{

}


PG70Gripper::~PG70Gripper() {
	// LWA powercube chain class takes care of closing the CAN bus
    close();
}

bool PG70Gripper::init()
{
	int ret = 0;
    int DOF = params_->GetDOF();
    std::vector<int> ModulIDs = params_->GetModuleIDs();
    int CanBaudrate = params_->GetBaudrate();
    std::vector<double> MaxVel = params_->GetMaxVel();
    std::vector<double> MaxAcc = params_->GetMaxAcc();
    std::vector<double> Offsets = params_->GetOffsets();
    std::vector<double> LowerLimits = params_->GetLowerLimits();
    std::vector<double> UpperLimits = params_->GetUpperLimits();
    std::vector<std::string> JointNames = params_->GetJointNames();

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

    std::cout << "Initializing PG70 gripper on the " << params_->getArmName().c_str();
    std:: cout << " arm, module id: " << params_->GetModuleID(0) << " handle: " << *m_CAN_handle<< std::endl;

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

        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getSerialNumber(*m_CAN_handle, params_->GetModuleID(i), &SerialNumber);
        pthread_mutex_unlock(m_CAN_mutex.get());
		if(ret!=0) return false;

        if((int)SerialNumber == (int)params_->GetSerialNumber())
		{

//			// home
//			pthread_mutex_lock(m_CAN_mutex.get());
//			ret = PCube_homeModule(*m_CAN_handle, ModulIDs[i]);
//			pthread_mutex_unlock(m_CAN_mutex.get());
//			if(ret<0)
//			{
//				ROS_ERROR("Error homing gripper of %s arm.", (params_->GetArmSelect().c_str()));
//				return false;
//			}
//
//
//			// wait until gripper is homed
//			unsigned long int state = 0;
//			while(!(state & PC_STATE_HOME_OK))
//			{
//				ros::Duration(0.5).sleep();
//				pthread_mutex_lock(m_CAN_mutex.get());
//				ret = PCube_getModuleState(*m_CAN_handle, ModulIDs[i], &state);
//				pthread_mutex_unlock(m_CAN_mutex.get());
//			}

			// reset
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error resetting gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}


			// max vel
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxVel(*m_CAN_handle, ModulIDs[i], MaxVel[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max vel of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			// max acceleration
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxAcc(*m_CAN_handle, ModulIDs[i], MaxAcc[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max acc of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}


			// Min pos
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMinPos(*m_CAN_handle, ModulIDs[i], (LowerLimits[i]));
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting min pos of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			// Max pos
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxPos(*m_CAN_handle, ModulIDs[i], (UpperLimits[i]));
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max pos of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}


			// reset
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error resetting gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			//get gripper pos
			float gripper_pos;
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_getModulePos(*m_CAN_handle, ModulIDs[i], &gripper_pos);
            pthread_mutex_unlock(m_CAN_mutex.get());

			if(ret<0)
			{
                ROS_ERROR("Error getting gripper pos of %s arm.", (params_->getArmName().c_str()));
				return false;
			}
            m_positions[i] = (((double)gripper_pos));

			//get gripper Max current
			float MaxCurrent;
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_getMaxCurrent(*m_CAN_handle, ModulIDs[i], &MaxCurrent);
            pthread_mutex_unlock(m_CAN_mutex.get());

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
            ROS_ERROR("Error gripper: Serial numbers don't coincide... %d %d", (int)SerialNumber, (int)params_->GetSerialNumber());
			return false;
		}
	}

	m_pc_status = PC_CTRL_OK;
	m_Initialized = true;
	m_CANDeviceOpened = true;
    ROS_INFO("Successfully initialized PG70 gripper on %s arm", params_->getArmName().c_str());

	return true;
}

bool PG70Gripper::recover()
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
    std::vector<int> ModulIDs = params_->GetModuleIDs();
    std::vector<double> MaxVel = params_->GetMaxVel();
    std::vector<double> MaxAcc = params_->GetMaxAcc();
    int DOF = params_->GetDOF();

	for(int i=0; i<DOF; i++)
	{

		// reset
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
        pthread_mutex_unlock(m_CAN_mutex.get());
		if(ret<0)
		{
            ROS_ERROR("Error resetting gripper of %s arm.", (params_->getArmName().c_str()));
			return false;
		}

//		// home
//		pthread_mutex_lock(m_CAN_mutex.get());
//		ret = PCube_homeModule(*m_CAN_handle, ModulIDs[i]);
//		pthread_mutex_unlock(m_CAN_mutex.get());
//		if(ret<0)
//		{
//			ROS_ERROR("Error homing gripper of %s arm.", (params_->GetArmSelect().c_str()));
//			return false;
//		}
//
//
//		// wait until gripper is homed
//		unsigned long int state = 0;
//		while(!(state & PC_STATE_HOME_OK))
//		{
//			ros::Duration(0.5).sleep();
//			pthread_mutex_lock(m_CAN_mutex.get());
//			ret = PCube_getModuleState(*m_CAN_handle, ModulIDs[i], &state);
//			pthread_mutex_unlock(m_CAN_mutex.get());
//		}
//
//		// reset
//		pthread_mutex_lock(m_CAN_mutex.get());
//		ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
//		pthread_mutex_unlock(m_CAN_mutex.get());
//		if(ret<0)
//		{
//			ROS_ERROR("Error resetting gripper of %s arm.", (params_->GetArmSelect().c_str()));
//			return false;
//		}
//
//
//		// max vel
//		if((target_vel<MaxVel[i]) && (target_vel>0.0))
//		{
//			pthread_mutex_lock(m_CAN_mutex.get());
//			ret = PCube_setMaxVel(*m_CAN_handle, ModulIDs[i], target_vel);
//			pthread_mutex_unlock(m_CAN_mutex.get());
//			if(ret<0)
//			{
//				ROS_ERROR("Error setting max vel of gripper of %s arm.", (params_->GetArmSelect().c_str()));
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
//		pthread_mutex_lock(m_CAN_mutex.get());
//		ret = PCube_setMaxAcc(*m_CAN_handle, ModulIDs[i], MaxAcc[i]);
//		pthread_mutex_unlock(m_CAN_mutex.get());
//		if(ret<0)
//		{
//			ROS_ERROR("Error setting max acc of gripper of %s arm.", (params_->GetArmSelect().c_str()));
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
    int DOF = params_->GetDOF();
    std::vector<int> ModulIDs = params_->GetModuleIDs();
	unsigned char dio;
	unsigned long int state;

	//get gripper pos
	for(int i=0; i<DOF; i++)
	{
		float gripper_pos;
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getStateDioPos(*m_CAN_handle, ModulIDs[i], &state, &dio, &gripper_pos);
        pthread_mutex_unlock(m_CAN_mutex.get());

		if(ret<0)
		{
            ROS_ERROR("Error getting gripper pos of %s arm.", (params_->getArmName().c_str()));
			return false;
		}
        m_positions[i] = (((double)gripper_pos));
		m_status[i] = state;
		m_dios[i] = dio;
	}

	return true;
}

bool PG70Gripper::movePos(double target_pos)
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
	float gripper_pos;
    double UpperLimit = params_->GetUpperLimits().at(0);
    double LowerLimit = params_->GetLowerLimits().at(0);

	if(!DoHoming())
	{
		return false;
	}
    if((target_pos < UpperLimit) &&  (target_pos > LowerLimit))
	{

        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_moveModulePos(*m_CAN_handle, params_->GetModuleID(0), (float)target_pos,
                                  &m_status[0], &m_dios[0], &gripper_pos);
        pthread_mutex_unlock(m_CAN_mutex.get());
        m_positions[0] = ((double)gripper_pos);
	}

	else
	{
        ROS_ERROR("Gripper position outside limits: %f [mm]", target_pos*1000.0);
		return false;
	}

	if(ret<0)
	{
		return false;
	}
    m_status[0] = m_status[0] | PC_STATE_MOTION;
    m_executing_pos_command = true;
	return true;
}

bool PG70Gripper::moveVel(double target_vel)
{
    PCTRL_CHECK_INITIALIZED();
    int ret = 0;
    float gripper_pos;
    double UpperLimit = params_->GetUpperLimits().at(0);
    double LowerLimit = params_->GetLowerLimits().at(0);

    double horizon = 0.003;
    double delta_t, target_time, target_time_horizon;
    unsigned short time4motion;

    double target_pos_horizon, delta_pos, delta_pos_horizon;


    delta_t = (ros::Time::now() - m_last_time_pub).toSec();
    m_last_time_pub = ros::Time::now();


    if (delta_t >= 0.050)
    {
        target_time = 0.050; //sec
    }
    else
    {
        target_time = delta_t;
    }

    target_time_horizon = target_time + horizon;

    time4motion = (unsigned short)((target_time_horizon)*1000.0);


    delta_pos_horizon = target_time_horizon*target_vel;
    target_pos_horizon = m_positions[0] + delta_pos_horizon;
    delta_pos = target_time*target_vel;

    if((target_pos_horizon < UpperLimit) &&  (target_pos_horizon > LowerLimit))
    {

        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_moveStepExtended(*m_CAN_handle,
                                     params_->GetModuleID(0),
                                     (float)target_pos_horizon,
                                     time4motion,
                                     &m_status[0],
                                     &m_dios[0],
                                     &gripper_pos);

        pthread_mutex_unlock(m_CAN_mutex.get());
        m_positions[0] = ((double)gripper_pos);
    }

    else
    {
        ROS_ERROR("Gripper position outside limits in velocity command: %f [mm]", target_pos_horizon*1000.0);
        return false;
    }

    if(ret<0)
    {
        return false;
    }

    return true;
}

bool PG70Gripper::CloseGripper(double target_vel, double current_limit)
{
	PCTRL_CHECK_INITIALIZED();
	int ret = 0;
    double MaxVel = params_->GetMaxVel().at(0);

	if(!DoHoming())
	{
		return false;
	}

    for(int i=0; i<params_->GetDOF(); i++)
	{
		if((current_limit < getMaxCurrent()) && (current_limit > 0))
		{
            pthread_mutex_lock(m_CAN_mutex.get());
            ret += PCube_setMaxCurrent(*m_CAN_handle, params_->GetModuleID(i), (float)current_limit);
            pthread_mutex_unlock(m_CAN_mutex.get());
		}

		else
		{
			ROS_ERROR("Exceeded max current limit of PG70 gripper");
			return false;
		}

		if((target_vel < MaxVel) && (target_vel > 0.0))
		{
            pthread_mutex_lock(m_CAN_mutex.get());
            ret += PCube_setMaxVel(*m_CAN_handle, params_->GetModuleID(i), (float)target_vel);
            pthread_mutex_unlock(m_CAN_mutex.get());
		}
		else
		{
			ROS_ERROR("Exceeded max vel limit of PG70 gripper");
			return false;
		}

	}
	if(ret<0) return false;

    return movePos(0.001);
}

bool PG70Gripper::DoHoming()
{
	int ret;
    int DOF = params_->GetDOF();
    std::vector<int> ModulIDs = params_->GetModuleIDs();
    std::vector<double> MaxVel = params_->GetMaxVel();
    std::vector<double> MaxAcc = params_->GetMaxAcc();
    std::vector<double> Offsets = params_->GetOffsets();
    std::vector<double> LowerLimits = params_->GetLowerLimits();
    std::vector<double> UpperLimits = params_->GetUpperLimits();

	PCTRL_CHECK_INITIALIZED();
	for(unsigned int i=0; i<DOF; i++)
	{
		unsigned long int state = 0;
        pthread_mutex_lock(m_CAN_mutex.get());
        ret = PCube_getModuleState(*m_CAN_handle, ModulIDs[i], &state);
        pthread_mutex_unlock(m_CAN_mutex.get());

		if(!(state & PC_STATE_HOME_OK))
		{

			// home
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_homeModule(*m_CAN_handle, ModulIDs[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error homing gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			// wait until gripper is homed
			state = 0;
			while(!(state & PC_STATE_HOME_OK))
			{
				ros::Duration(0.5).sleep();
                pthread_mutex_lock(m_CAN_mutex.get());
                ret = PCube_getModuleState(*m_CAN_handle, ModulIDs[i], &state);
                pthread_mutex_unlock(m_CAN_mutex.get());
			}

			// reset
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error resetting gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}



			// max vel
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxVel(*m_CAN_handle, ModulIDs[i], MaxVel[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max vel of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			// max acceleration
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxAcc(*m_CAN_handle, ModulIDs[i], MaxAcc[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max acc of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}


			// Min pos
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMinPos(*m_CAN_handle, ModulIDs[i], (LowerLimits[i]));
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting min pos of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			// Max pos
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_setMaxPos(*m_CAN_handle, ModulIDs[i], (UpperLimits[i]));
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error setting max pos of gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}


			// reset
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_resetModule(*m_CAN_handle, ModulIDs[i]);
            pthread_mutex_unlock(m_CAN_mutex.get());
			if(ret<0)
			{
                ROS_ERROR("Error resetting gripper of %s arm.", (params_->getArmName().c_str()));
				return false;
			}

			//get gripper pos
			float gripper_pos;
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_getModulePos(*m_CAN_handle, ModulIDs[i], &gripper_pos);
            pthread_mutex_unlock(m_CAN_mutex.get());

			if(ret<0)
			{
                ROS_ERROR("Error getting gripper pos of %s arm.", (params_->getArmName().c_str()));
				return false;
			}
            m_positions[i] = (((double)gripper_pos));

			//get gripper Max current
			float MaxCurrent;
            pthread_mutex_lock(m_CAN_mutex.get());
            ret = PCube_getMaxCurrent(*m_CAN_handle, ModulIDs[i], &MaxCurrent);
            pthread_mutex_unlock(m_CAN_mutex.get());

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

bool PG70Gripper::executingPosCommand()
{
    bool in_motion = m_status[0] & PC_STATE_MOTION;

    if(m_executing_pos_command && in_motion)
        return true;

    m_executing_pos_command = false;
    return false;
}

