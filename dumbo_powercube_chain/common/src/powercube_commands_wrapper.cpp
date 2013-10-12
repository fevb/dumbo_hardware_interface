#include <dumbo_powercube_chain/powercube_commands_wrapper.h>
#include <string>
#include <iostream>
extern "C"{
#include <stdio.h>
#include <dumbo_powercube_chain/powercube_commands.h>
#include <dumbo_powercube_chain/powercube_defines.h>
#include <dumbo_powercube_chain/utils.h>
// #include <dumbo_powercube_chain/dumbo_kinematics.h>
// #include <dumbo_powercube_chain/dumbo_arm_params.h>
// #include <dumbo_powercube_chain/collision.h>
}
#include <time.h>

std::string m_ArmSelect;

void PCube_closeDevice(canHandle DeviceHandle)
{

	canBusOff(DeviceHandle);
	(void)canClose(DeviceHandle);

}

int PCube_openDevice(canHandle *DeviceHandle, int *CAN_Channel, std::string InitStr)
{

	int channel;
	int ret = -1;
	int found = 0;
	unsigned long int serial;
	unsigned char msg[8];
	unsigned char ret_msg[8];

	int num_channels = 0;
	canHandle h;

	m_ArmSelect = InitStr;

	ret = canGetNumberOfChannels(&num_channels);
	if(ret<0)
	{
		std::cout << "Error getting number of CAN channels." << std::endl;
		return ret;
	}
	std::cout << "Number of CAN channels: " << num_channels << std::endl;
	std::cout << "Looking for dumbo..." << std::endl;

	// Find arm
	for(channel=0; channel<num_channels && found==0; channel++)
	{
		struct module test_cube;
		int bussAdress;

		h = canOpenChannel(channel, canWANT_EXCLUSIVE |canWANT_EXTENDED);
		ret = (int)canSetBusParams(h, BAUD_500K,4,3,1,1,0);
		if(ret<0)
		{
			std::cout << "Error setting bus parameters..." << std::endl;
			PCube_closeDevice(h);
//			return ret;
		}

		else
		{
			canSetBusOutputControl(h, canDRIVER_NORMAL);
			ret = (int)canBusOn(h);
			if(ret<0)
			{
				std::cout << "Error turning on can bus..." << std::endl;
				PCube_closeDevice(h);
//				return ret;
			}

			else
			{

				test_cube.handle = h;
				test_cube.canID = 1;
				msg[0] = PC_COMMAND_GET_EXTENDED;
				msg[1] = PC_PARA_DEF_ADDRESS;
				pc_send_command_to_module(test_cube, msg, 2);
				bussAdress = pc_listen_for_response(h,&msg);
				printf("can channel:%d, response:%d\n",channel,bussAdress);
				if(bussAdress==1)
				{
					msg[0] = PC_COMMAND_GET_EXTENDED;
					msg[1] = PC_PARA_DEF_CUBE_SERIAL;
					pc_request_value_from_module(test_cube, msg);
					pc_listen_for_response(test_cube.handle, &ret_msg);

					get_data_uint32(ret_msg,&serial);
					printf("BussAdress %d has serial %u (left arm:%u   right arm:%u)\n",
							bussAdress,(unsigned int)serial,
							DEF_SCHUNK_LEFTARM_SERIAL_NUMBER,
							DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER);


					if(m_ArmSelect=="left")
					{
						if((int)serial == DEF_SCHUNK_LEFTARM_SERIAL_NUMBER)
						{
							printf("Connected to Dumbo left arm! \n");
							found+=1;
							*DeviceHandle = h;
							*CAN_Channel = channel;
							pc_listen_for_response(h,&msg); // clears CAN buffer
						}
						else
						{
							pc_listen_for_response(h,&msg); // clears CAN buffer
							PCube_closeDevice(h);
						}
					}
					else if(m_ArmSelect=="right")
					{
						if((int)serial == DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER)
						{
							printf("Connected to Dumbo right arm! \n");
							found+=1;
							*DeviceHandle = h;
							*CAN_Channel = channel;
							pc_listen_for_response(h,&msg); // clears CAN buffer
						}
						else
						{
							pc_listen_for_response(h,&msg); // clears CAN buffer
							PCube_closeDevice(h);
						}
					}
				}

				else
				{
					pc_listen_for_response(h,&msg);
					PCube_closeDevice(h);
				}
			}
		}
	}

	if(found==0)
	{
		std::cout << m_ArmSelect << " arm not found." << std::endl;
		ret = -1;
		return ret;
	}


	pc_listen_for_response(h, &msg);
	ret = 0;

	return ret;
}


int PCube_resetAll(canHandle DeviceHandle)
{
	int ret = 0;
	pc_reset_all_on_bus(DeviceHandle);
	return ret;
}

int PCube_resetModule(canHandle DeviceHandle, int ModulID)
{
	int ret = 0;
	struct module dumbo_cube;
	char ret_msg[8];
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModulID;

	pc_reset_module(dumbo_cube);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModulID)){
		ret = -1;
	}

	return ret;
}

int PCube_setMinPos(canHandle DeviceHandle, int ModuleID, float LowerLimit)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float lowerlim = LowerLimit;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	// set limits
	msg[0]=PC_COMMAND_SET_EXTENDED;
	msg[1]=PC_PARA_MIN_POS;
	dlc = set_data_float(msg,&lowerlim);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	ret_msg[2]=0;
	pc_listen_for_response(dumbo_cube.handle, &ret_msg);
	if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
			ret_msg[1]==PC_PARA_MIN_POS &&
			ret_msg[2]==0X64)){
		ret += -1;
		std::cout << "Set min lim error" << std::endl;
	}

	return ret;
}


int PCube_setMaxPos(canHandle DeviceHandle, int ModuleID, float UpperLimit)
{

	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float upperlim = UpperLimit;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	// set limits
	msg[0]=PC_COMMAND_SET_EXTENDED;
	msg[1]=PC_PARA_MAX_POS;
	dlc = set_data_float(msg,&upperlim);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	ret_msg[2]=0;
	pc_listen_for_response(dumbo_cube.handle, &ret_msg);
	if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
			ret_msg[1]==PC_PARA_MAX_POS &&
			ret_msg[2]==0X64)){
		ret += -1;
		std::cout << "Set max lim error" << std::endl;
	}

	return ret;
}


int PCube_haltAll(canHandle DeviceHandle)
{
	int ret = 0;
	(void)pc_halt_all_on_bus(DeviceHandle);
	return ret;
}

int PCube_haltModule(canHandle DeviceHandle, int ModulID)
{
	int ret = 0;
	struct module dumbo_cube;
	dumbo_cube.canID = ModulID;
	dumbo_cube.handle = DeviceHandle;
	(void) pc_halt_module(dumbo_cube);
	return ret;
}


int PCube_setMaxVel(canHandle DeviceHandle, int ModuleID, float maxVelocity)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float fvalue;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	// *** should be max vel, not target vel
	//set vel
	msg[0]=PC_COMMAND_SET_EXTENDED;
	msg[1]=PC_PARA_TARGET_VEL;
	fvalue = maxVelocity;//20*PI/180; // a fast value, 20 deg/s
	dlc = set_data_float(msg,&fvalue);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModuleID)){
		ret = -1;
	}

	return ret;
}

int PCube_setMaxAcc(canHandle DeviceHandle, int ModuleID, float maxAcceleration)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float fvalue;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	//*** should be max acc not target acc
	//set vel
	msg[0]=PC_COMMAND_SET_EXTENDED;
	msg[1]=PC_PARA_TARGET_ACC;
	fvalue = maxAcceleration;//20*PI/180; // a fast value, 20 deg/s
	dlc = set_data_float(msg,&fvalue);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModuleID)){
		ret = -1;
	}

	return ret;
}

int PCube_getMaxCurrent(canHandle DeviceHandle, int ModuleID, float *maxCurrent)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float fvalue;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	//*** should be max acc not target acc
	//set vel
	msg[0]=PC_COMMAND_GET_EXTENDED;
	msg[1]=PC_PARA_MAX_CUR;
	pc_request_value_from_module(dumbo_cube, msg);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModuleID)){
		ret = -1;
		return ret;
	}
	else
	{
		get_data_float(ret_msg, maxCurrent);
	}

	return ret;


}

int PCube_setMaxCurrent(canHandle DeviceHandle, int ModuleID, float maxCurrent)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	unsigned int dlc;
	float fvalue;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	//*** should be max acc not target acc
	//set vel
	msg[0]=PC_COMMAND_SET_EXTENDED;
	msg[1]=PC_PARA_MAX_CUR;
	fvalue = maxCurrent;
	dlc = set_data_float(msg,&fvalue);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModuleID)){
		ret = -1;
	}

	return ret;
}

int PCube_getConfig(canHandle DeviceHandle, int ModuleID, unsigned long int *confword)
{
	int ret = 0;
	unsigned char msg[8];

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	msg[0] = PC_COMMAND_GET_EXTENDED;
	msg[1] = PC_PARA_CONFIG;
	pc_request_value_from_module(dumbo_cube, msg);
	ret = pc_listen_for_response(dumbo_cube.handle,&msg);
	get_data_uint32(msg, confword);
	return ret;
}

int PCube_setConfig(canHandle DeviceHandle, int ModuleID, unsigned long int confword)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned int dlc;

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	unsigned long int config_word = confword;
	msg[0] = PC_COMMAND_SET_EXTENDED;
	msg[1] = PC_PARA_CONFIG;
	dlc = set_data_uint32(msg, &config_word);
	pc_send_command_to_module(dumbo_cube, msg, dlc);
	ret = pc_listen_for_response(dumbo_cube.handle,&msg);

	return ret;
}

int PCube_getStateDioPos(canHandle DeviceHandle, int ModuleID, unsigned long int *state,
		unsigned char *dio, float *position)
{

	int ret = 0;

	// *** have to implement dio
	*dio = 0;

	ret+=PCube_getModulePos(DeviceHandle, ModuleID, position);
	//  m_last_angle[ModuleID-1] = (double)cur_pos;

	ret+=PCube_getModuleState(DeviceHandle, ModuleID, state);

	return ret;
}

int PCube_getSerialNumber(canHandle DeviceHandle, int ModuleID, unsigned long int *SerialNumber)
{
	int channel;
	int ret = 0;
	int found = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];

	struct module test_cube;
	test_cube.handle = DeviceHandle;
	test_cube.canID = ModuleID;

	msg[0] = PC_COMMAND_GET_EXTENDED;
	msg[1] = PC_PARA_DEF_CUBE_SERIAL;
	pc_request_value_from_module(test_cube, msg);
	if(pc_listen_for_response(test_cube.handle, &ret_msg)!=ModuleID)
	{
		ret = -1;
		return ret;
	}

	get_data_uint32(ret_msg, SerialNumber);


	return ret;
}

int PCube_homeModule(canHandle DeviceHandle, int ModuleID)
{
	int ret = 0;
	unsigned long int state;
	struct module dumbo_cube;
	unsigned char msg[8];
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;


	ret = PCube_getModuleState(DeviceHandle, ModuleID, &state);

	if(!(state & PC_STATE_HOME_OK) && ret==0)
	{
		pc_home_module(dumbo_cube);
		if(pc_listen_for_response(dumbo_cube.handle,&msg) != ModuleID)
		{
			ret += -1;
			std::cout << "Error homing module" << ModuleID << std::endl;
		}
	}
	return ret;
}

int PCube_getModuleState(canHandle DeviceHandle, int ModuleID, unsigned long int *state)
{
	int ret = 0;
	unsigned char msg[8];
	unsigned char ret_msg[8];

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	msg[0] = PC_COMMAND_GET_EXTENDED;
	msg[1] = PC_PARA_CUBE_STATE;

	pc_request_value_from_module(dumbo_cube, msg);
	if( pc_listen_for_response(dumbo_cube.handle, &ret_msg) != ModuleID )
	{
		ret += -1;
		std::cout << "Error getting state of module " << ModuleID << std::endl;
	}
	else
	{
		get_data_uint32(ret_msg,state);
	}

	return ret;
}

int PCube_getModulePos(canHandle DeviceHandle, int ModuleID, float *pos)
{

	int ret = 0;
	float cur_pos;
	unsigned char msg[8];
	unsigned char ret_msg[8];

	struct module dumbo_cube;
	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;

	msg[0] = PC_COMMAND_GET_EXTENDED;
	msg[1] = PC_PARA_ACT_POS;

	pc_request_value_from_module(dumbo_cube, msg);
	if(pc_listen_for_response(dumbo_cube.handle,&ret_msg)!=(ModuleID))
	{
		ret = -1;
		std::cout << "Error getting pos of module " << ModuleID << std::endl;
	}
	else
	{
		get_data_float(ret_msg,&(cur_pos));
		*pos = PCube_correctAngle(ModuleID, cur_pos);
	}


	return ret;

}

int PCube_movePos(canHandle DeviceHandle, const std::vector<int> &ModuleIDs, const std::vector<float> &target_pos, std::vector<float> &pos)
{
	int ret = 0;

	struct timespec tsleep;
	tsleep.tv_sec = 0;
	tsleep.tv_nsec = 1000000;
	unsigned long int state;
	unsigned char dio;

	if(ModuleIDs.size() != target_pos.size())
	{
		ret = -1;
		return ret;
	}

	pos.resize(target_pos.size());
	float pos_;

	for(int i=0; i<(int)ModuleIDs.size(); i++)
	{
		nanosleep(&tsleep, NULL);
		PCube_moveModulePos(DeviceHandle, ModuleIDs.at(i), target_pos.at(i), &state, &dio, &pos_);
		pos[i] = pos_;
	}

	return ret;
}

int PCube_moveModulePos(canHandle DeviceHandle, int ModuleID, const float target_pos, unsigned long int *ShortState, unsigned char *dio, float *pos)
{
	int ret = 0;
	int res;
	unsigned int dlc;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	float fangle;

	struct module dumbo_cube;


	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;


	msg[0] = PC_COMMAND_SET_MOTION;
	msg[1] = PC_MOTION_FRAMP_ACK;
	fangle = PCube_correctAngle(ModuleID, target_pos);
	dlc = set_data_float(msg, &fangle);
	pc_send_command_to_module(dumbo_cube, msg, dlc);

	if((res=pc_listen_for_response(dumbo_cube.handle, &ret_msg))!=ModuleID)
	{
		ret += -1;
	}
	else
	{
		get_data_float(ret_msg,&fangle);
	}

	*ShortState = *ShortState | PCube_Process_ShortState(ret_msg[6]);
	*dio = ret_msg[7];
	*pos = PCube_correctAngle(ModuleID, fangle);
	return ret;
}


int PCube_moveStepExtended(canHandle DeviceHandle, int ModuleID, float target_pos, unsigned short target_time,
		unsigned long int *ShortState, unsigned char *dio, float *pos)
{

	int ret = 0;
	int res;
	unsigned int dlc;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	float fangle;

	struct module dumbo_cube;

	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;


	msg[0] = PC_COMMAND_SET_MOTION;
	msg[1] = PC_MOTION_FSTEP_ACK;
	fangle = PCube_correctAngle(ModuleID, target_pos);

	unsigned short int target_time_ = target_time;
	dlc = set_data_float_uint16(msg, &fangle, &target_time_);
	pc_send_command_to_module(dumbo_cube, msg, dlc);

	if((res=pc_listen_for_response(dumbo_cube.handle, &ret_msg))!=ModuleID)
	{
		ret += -1;
	}
	else
	{
		get_data_float(ret_msg,&fangle);
	}

	*ShortState = *ShortState | PCube_Process_ShortState(ret_msg[6]);
	*dio = ret_msg[7];
	*pos = PCube_correctAngle(ModuleID, fangle);
	return ret;
}

int PCube_moveModuleVel(canHandle DeviceHandle, int ModuleID, float target_vel, unsigned long int *ShortState, unsigned char *dio, float *pos)
{

	int ret = 0;
	int res;
	unsigned int dlc;
	unsigned char msg[8];
	unsigned char ret_msg[8];
	float fvel;
	float fangle;

	struct module dumbo_cube;

	dumbo_cube.handle = DeviceHandle;
	dumbo_cube.canID = ModuleID;


	msg[0] = PC_COMMAND_SET_MOTION;
	msg[1] = PC_MOTION_FVEL_ACK;
	fvel = PCube_correctVel(ModuleID, target_vel);

	dlc = set_data_float(msg, &fvel);
	pc_send_command_to_module(dumbo_cube, msg, dlc);

	if((res=pc_listen_for_response(dumbo_cube.handle, &ret_msg))!=ModuleID)
	{
		ret += -1;
	}
	else
	{
		get_data_float(ret_msg,&fangle);
	}

	*ShortState = *ShortState | PCube_Process_ShortState(ret_msg[6]);
	*dio = ret_msg[7];
	*pos = PCube_correctAngle(ModuleID, fangle);
	return ret;
}

int PCube_startMotionAll(canHandle DeviceHandle)
{
	int ret=0;
	pc_sync_all_on_bus(DeviceHandle);
	return ret;
}

std::vector<float> PCube_correctAngles(const std::vector<float> &angles)
{
	std::vector<float> angles_c = angles;

	angles_c[0] *= -1;
	angles_c[2] *= -1;
	angles_c[3] *= -1;
	angles_c[4] *= -1;
	angles_c[5] *= -1;
	angles_c[6] *= -1;

	for(int i=0;i<(int)angles_c.size();i++){
		while(angles_c[i]>PI){
			angles_c[i] -= (2*PI);
		}
		while(angles_c[i] < (-PI)){
			angles_c[i] += (2*PI);
		}
	}

	return angles_c;
}

float PCube_correctAngle(int ModuleID, const float angle)
{
	float angle_c = angle;

	// *** in case of gripper return same angle
	if(ModuleID==8)
	{
		return angle_c;
	}


	if(ModuleID!=2)
	{
		angle_c *= -1.0;
	}

	while(angle_c > PI){
		angle_c -= (2*PI);
	}
	while(angle_c < (-PI)){
		angle_c += (2*PI);
	}

	return angle_c;
}

std::vector<float> PCube_correctVel(const std::vector<float> &vel)
{
	std::vector<float> vel_c = vel;

		vel_c[0] *= -1.0;
		vel_c[2] *= -1.0;
		vel_c[3] *= -1.0;
		vel_c[4] *= -1.0;
		vel_c[5] *= -1.0;
		vel_c[6] *= -1.0;

		return vel_c;
}

float PCube_correctVel(int ModuleID, const float vel)
{
	// *** in case of gripper return same velocity
	if(ModuleID==8)
	{
		return vel;
	}

	if(ModuleID!=2)
	{
		return vel*-1.0;
	}

	return vel;
}


unsigned long int PCube_Process_ShortState(char ShortState)
{
	unsigned long int state = 0;

	if(ShortState & PC_ACK_SHORT_NOT_OK) state = state | PC_STATE_ERROR;
	if(ShortState & PC_ACK_SHORT_SWR) state = state | PC_STATE_SWR;
	if(ShortState & PC_ACK_SHORT_SW1) state = state | PC_STATE_SW1;
	if(ShortState & PC_ACK_SHORT_SW2) state = state | PC_STATE_SW2;
	if(ShortState & PC_ACK_SHORT_MOTION) state = state | PC_STATE_MOTION;
	if(ShortState & PC_ACK_SHORT_RAMP_END) state = state | PC_STATE_RAMP_END;
	if(ShortState & PC_ACK_SHORT_INPROGRESS) state = state | PC_STATE_IN_PROGRESS;
	if(ShortState & PC_ACK_SHORT_FULLBUFFER) state = state | PC_STATE_FULLBUFFER;

	return state;
}
