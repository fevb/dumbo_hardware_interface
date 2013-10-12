/*
 * PG70Gripper.h
 *
 *  Created on: Aug 10, 2012
 *      Author: francisco
 */

#ifndef PG70GRIPPER_H_
#define PG70GRIPPER_H_

#include <dumbo_powercube_chain/PowerCubeCtrl.h>

class PG70Gripper: public PowerCubeCtrl
{
public:
	PG70Gripper(PowerCubeCtrlParams *params);

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

	bool MovePos(double target_pos);

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

	bool inMotion();

private:
	double m_MaxCurrent;
	bool m_motion;


};

#endif /* PG70GRIPPER_H_ */
