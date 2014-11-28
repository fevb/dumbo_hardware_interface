/*
 *  ForceTorqueSensor.cpp
 *
 *  Created on: Aug 3, 2012
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
#include <iostream>
#include <string>
#include <iomanip>
#include <dumbo_force_torque_sensor/ForceTorqueSensor.h>
extern "C"{
#include <dumbo_powercube_chain/powercube_commands.h>
}

#define FT_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    return false;														\
}


ForceTorqueSensor::ForceTorqueSensor()
{

	m_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;

	m_CANDeviceOpened = false;
	m_Initialized = false;


//	m_tf_listener = new tf::TransformListener(ros::Duration(10.0));

}


ForceTorqueSensor::~ForceTorqueSensor() {
    disconnect();
}


bool ForceTorqueSensor::init(const std::string &serial_number,
                             const std::string &arm_name)
{

    m_serial_number = serial_number;
    m_arm_name = arm_name;

	canHandle h;
	int ret = -1;
	int channel;
	int num_channels;
	int found = 0;
	unsigned char msg[8];

	if(isInitialized())
	{
		ROS_WARN("F/T sensor already initialized");
		return false;
	}

	ROS_INFO("----------------------------------------------");
    ROS_INFO("Initializing %s arm F/T sensor.", m_arm_name.c_str());
    ROS_INFO("Serial Number: %s", m_serial_number.c_str());


	ret = canGetNumberOfChannels(&num_channels);
	if(ret<0)
	{
		std::cout << "Error getting number of CAN channels." << std::endl;
		return ret;
	}

	for(channel=0; channel<num_channels && found==0; channel++)
	{
		pthread_mutex_lock(&m_CAN_mutex);
		h = canOpenChannel(channel, canWANT_EXCLUSIVE);
		ret = canSetBusParams(h, BAUD_250K, 0, 0, 0, 0, 0);
		pthread_mutex_unlock(&m_CAN_mutex);

		m_DeviceHandle = h;
		m_CANDeviceOpened = true;

		if(ret<0)
		{
            disconnect();
		}

		else
		{
			pthread_mutex_lock(&m_CAN_mutex);
			canSetBusOutputControl(h, canDRIVER_NORMAL);
			canBusOn(h);
			pthread_mutex_unlock(&m_CAN_mutex);

			char serial_char[8];
			std::string serial_number;
			pthread_mutex_lock(&m_CAN_mutex);
			get_Serial_Number(h, true, serial_char);
			pthread_mutex_unlock(&m_CAN_mutex);
			serial_number.assign(serial_char, 6);

            if(serial_number!=m_serial_number)
			{
                pthread_mutex_lock(&m_CAN_mutex);
				pc_listen_for_response(h,&msg);
                pthread_mutex_unlock(&m_CAN_mutex);
                disconnect();
			}

			else
			{
				found++;
				m_CAN_Channel = channel;
                ROS_INFO("Found %s arm F/T sensor on channel: %d", m_arm_name.c_str(), channel);
			}
		}
	}


	if(found==0)
	{
        ROS_ERROR("%s arm F/T sensor not found", m_arm_name.c_str());
		return false;
	}


	get_Transducer_CalMatrix(h,&m_Calibration_Matrix[0][0]);

	std::cout << "Calibration matrix:" << std::endl;
	for(int i = 0; i<6; i++)
	{
		for(int j = 0; j<6; j++)
		{
			printf("%10.8f ", m_Calibration_Matrix[i][j]);
		}
		printf("\n");
	}

	m_Initialized = true;
	return true;
}

bool ForceTorqueSensor::disconnect(){
	if(m_CANDeviceOpened)
	{
        pthread_mutex_lock(&m_CAN_mutex);
        canBusOff(m_DeviceHandle);
		(void)canClose(m_DeviceHandle);
        pthread_mutex_unlock(&m_CAN_mutex);
		m_CANDeviceOpened = false;
        m_Initialized = false;
        return true;
    }
    return false;
}

bool ForceTorqueSensor::getFT(std::vector<double> &force, std::vector<double> &torque)
{
    FT_CHECK_INITIALIZED();
	double ft[6];
	int ret_val,i,j;
	signed short int s_out_short_int[7];
	signed short int SG[6];

	pthread_mutex_lock(&m_CAN_mutex);
	ret_val = get_SG_data(m_DeviceHandle, true, &s_out_short_int[0]);
	pthread_mutex_unlock(&m_CAN_mutex);

	if(ret_val<0)
	{
        ROS_ERROR("Error reading F/T sensor %s arm.", m_arm_name.c_str());
		return false;
	}

	// For some reason, the SG's are in strange order. this fixes:
	SG[0] = s_out_short_int[1];
	SG[1] = s_out_short_int[4];
	SG[2] = s_out_short_int[2];
	SG[3] = s_out_short_int[5];
	SG[4] = s_out_short_int[3];
	SG[5] = s_out_short_int[6];


	// Calculate F/T via cal matrix:
	for(i=0;i<6;i++)
	{
		ft[i] = 0;
		for(j=0;j<6;j++)
		{
			ft[i] += ((double)SG[j])*m_Calibration_Matrix[i][j];
		}
//		ft[i] += m_Bias[i];
	}

	//for(i=0; i<6; i++)std::cout << ft[i]

	// correct axis definitions
    if(m_arm_name=="left") // left arm
	{
        force[0] = -1*ft[1];
        force[1] = ft[0];
        force[2] =  ft[2];
        torque[0] = -1*ft[4];
        torque[1] = ft[3];
        torque[2] = ft[5];
	}

	else
	{
        force[0] = ft[1];
        force[1] = -1*ft[0];
        force[2] = ft[2];
        torque[0] = ft[4];
        torque[1] = -1*ft[3];
        torque[2] = ft[5];
	}


	return true;
}

bool ForceTorqueSensor::requestFT()
{
    FT_CHECK_INITIALIZED();
    int ret_val;

    pthread_mutex_lock(&m_CAN_mutex);
    ret_val = request_SG_data(m_DeviceHandle, false);
    pthread_mutex_unlock(&m_CAN_mutex);

    if(ret_val<0)
    {
        ROS_ERROR("Error reading F/T sensor %s arm.", m_arm_name.c_str());
        return false;
    }

    return true;

}

bool ForceTorqueSensor::readFT(std::vector<double> &force, std::vector<double> &torque)
{
    FT_CHECK_INITIALIZED();
    double ft[6];
    int ret_val,i,j;
    signed short int s_out_short_int[7];
    signed short int SG[6];

    pthread_mutex_lock(&m_CAN_mutex);
    ret_val = read_SG_data(m_DeviceHandle, false, &s_out_short_int[0]);
    pthread_mutex_unlock(&m_CAN_mutex);

    if(ret_val<0)
    {
        ROS_ERROR("Error reading F/T sensor %s arm.", m_arm_name.c_str());
        return false;
    }

    // For some reason, the SG's are in strange order. this fixes:
    SG[0] = s_out_short_int[1];
    SG[1] = s_out_short_int[4];
    SG[2] = s_out_short_int[2];
    SG[3] = s_out_short_int[5];
    SG[4] = s_out_short_int[3];
    SG[5] = s_out_short_int[6];


    // Calculate F/T via cal matrix:
    for(i=0;i<6;i++)
    {
        ft[i] = 0;
        for(j=0;j<6;j++)
        {
            ft[i] += ((double)SG[j])*m_Calibration_Matrix[i][j];
        }
//		ft[i] += m_Bias[i];
    }

    //for(i=0; i<6; i++)std::cout << ft[i]

    // correct axis definitions
    if(m_arm_name=="left") // left arm
    {
        force[0] = -1*ft[1];
        force[1] = ft[0];
        force[2] =  ft[2];
        torque[0] = -1*ft[4];
        torque[1] = ft[3];
        torque[2] = ft[5];
    }

    else
    {
        force[0] = ft[1];
        force[1] = -1*ft[0];
        force[2] = ft[2];
        torque[0] = ft[4];
        torque[1] = -1*ft[3];
        torque[2] = ft[5];
    }


    return true;
}
