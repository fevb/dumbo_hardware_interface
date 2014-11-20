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


#ifndef PG70GRIPPER_H_
#define PG70GRIPPER_H_

#include <dumbo_powercube_chain/PowerCubeCtrl.h>

class PG70Gripper: public PowerCubeCtrl
{
public:
    PG70Gripper(boost::shared_ptr<PowerCubeCtrlParams> params,
                boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                boost::shared_ptr<canHandle> CAN_handle);

	virtual ~PG70Gripper();

	bool CloseDevice()
	{
		if(m_CANDeviceOpened)
		{
			m_Initialized = false;
			m_CANDeviceOpened = false;
			return true;
		}

		else
		{
			return false;
		}
	}

	// it is assumed that CAN communication has started with PowerCubeCtrl object
	bool Init();

	bool Recover();

	bool updateStates();

    bool movePos(double target_pos);

    bool moveVel(double target_vel);

	bool CloseGripper(double target_vel, double current_limit);

	double getMaxCurrent()
	{
		return m_MaxCurrent;
	}

	void setMaxCurrent(double MaxCurrent)
	{
		m_MaxCurrent = MaxCurrent;
	}

	bool DoHoming();

    bool executingPosCommand();

private:
    double m_MaxCurrent;
    bool m_executing_pos_command;


};

#endif /* PG70GRIPPER_H_ */
