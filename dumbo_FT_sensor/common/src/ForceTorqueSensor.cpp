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
#include <dumbo_force_torque/ForceTorqueSensor.h>
extern "C"{
#include <dumbo_powercube_chain/powercube_commands.h>
}

#define FT_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    return false;														\
}

using namespace Eigen;

ForceTorqueSensor::ForceTorqueSensor(std::string Serial_Number,
		std::string ArmSelect,
		std::vector<double> Bias,
		double EndEffectorMass)
{

	m_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;

	m_CANDeviceOpened = false;
	m_Initialized = false;

	m_SerialNumber = Serial_Number;
	m_ArmSelect = ArmSelect;
	m_sensor_frame_id = "/"+m_ArmSelect+"_arm_FT_sensor";

	m_b = Vector3d::Zero();
	m_a = Vector3d::Zero();


	setBias(Bias);
	setEndEffectorMass(EndEffectorMass);

//	m_tf_listener = new tf::TransformListener(ros::Duration(10.0));

}


ForceTorqueSensor::~ForceTorqueSensor() {
	Disconnect();
}


bool ForceTorqueSensor::Init(){

	canHandle h;
	int ret = -1;
	int channel;
	int num_channels;
	int found = 0;
	unsigned char msg[8];

	if(isInitialized())
	{
		ROS_WARN("FT sensor already initialized");
		return false;
	}

	ROS_INFO("----------------------------------------------");
	ROS_INFO("Initializing %s arm FT sensor.", m_ArmSelect.c_str());
	ROS_INFO("Serial Number: %s", m_SerialNumber.c_str());
	ROS_INFO("Bias: [%f %f %f %f %f %f]", m_Bias[0],
			m_Bias[1],
			m_Bias[2],
			m_Bias[3],
			m_Bias[4],
			m_Bias[5]);


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
			Disconnect();
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

			if(serial_number!=m_SerialNumber)
			{
				pc_listen_for_response(h,&msg);
				Disconnect();
			}

			else
			{
				found++;
				m_CAN_Channel = channel;
				ROS_INFO("Found %s arm FT sensor on channel: %d", m_ArmSelect.c_str(), channel);
			}
		}
	}


	if(found==0)
	{
		ROS_ERROR("%s arm FT sensor not found", m_ArmSelect.c_str());
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

void ForceTorqueSensor::Disconnect(){
	if(m_CANDeviceOpened)
	{
		pthread_mutex_lock(&m_CAN_mutex);
		canBusOff(m_DeviceHandle);
		(void)canClose(m_DeviceHandle);
		pthread_mutex_unlock(&m_CAN_mutex);
		m_CANDeviceOpened = false;
		m_Initialized = false;
	}
}

bool ForceTorqueSensor::Get_FT(geometry_msgs::Wrench &FT_raw)
{
	FT_CHECK_INITIALIZED();
	geometry_msgs::Wrench FT_measurement;
	double ft[6];
	int ret_val,i,j;
	signed short int s_out_short_int[7];
	signed short int SG[6];

	pthread_mutex_lock(&m_CAN_mutex);
	ret_val = get_SG_data(m_DeviceHandle, true, &s_out_short_int[0]);
	pthread_mutex_unlock(&m_CAN_mutex);

	if(ret_val<0)
	{
		ROS_ERROR("Error reading FT sensor %s arm.", m_ArmSelect.c_str());
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
	if(m_ArmSelect=="left") // left arm
	{
		FT_raw.force.x = -1*ft[1] + m_Bias[0];
		FT_raw.force.y = ft[0] + m_Bias[1];
		FT_raw.force.z = ft[2] + m_Bias[2];
		FT_raw.torque.x = -1*ft[4] + m_Bias[3];
		FT_raw.torque.y = ft[3] + m_Bias[4];
		FT_raw.torque.z = ft[5] + m_Bias[5];
	}

	else
	{
		FT_raw.force.x = ft[1] + m_Bias[0];
		FT_raw.force.y = -1*ft[0] + m_Bias[1];
		FT_raw.force.z = ft[2] + m_Bias[2];
		FT_raw.torque.x = ft[4] + m_Bias[3];
		FT_raw.torque.y = -1*ft[3] + m_Bias[4];
		FT_raw.torque.z = ft[5] + m_Bias[5];
	}


	return true;
}

bool ForceTorqueSensor::SetLPFCoeff(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
	if(a==Vector3d::Zero()||b==Vector3d::Zero())
	{
		ROS_ERROR("Error setting LPF coefficients");
		return false;
	}

	m_a = a;
	m_b = b;

	return true;
}

void ForceTorqueSensor::GetLPFCoeff(Eigen::Vector3d &a, Eigen::Vector3d &b)
{
	a = m_a;
	b = m_b;
}

// Compensates FT measurement with weight of end effector and publishes the
// measurement expressed in the arm_base_link frame
bool ForceTorqueSensor::Compensate(const geometry_msgs::WrenchStamped &FT_raw,
		const geometry_msgs::Vector3Stamped &gravity,
		geometry_msgs::WrenchStamped &FT_comp)
{

//	geometry_msgs::Vector3Stamped gravity;
//	gravity.header.stamp = ros::Time();
//	gravity.header.frame_id = "/arm_base_link";
//	gravity.vector.x = 0;
//	gravity.vector.y = 0;
//	gravity.vector.z = -9.81;

	if(FT_raw.header.frame_id != m_sensor_frame_id)
	{
		ROS_ERROR("FT raw measurement's frame different than m_sensor_frame_id");
		return false;
	}

	geometry_msgs::Vector3Stamped gravity_ = gravity;
	gravity_.header.stamp = ros::Time();

	geometry_msgs::Vector3Stamped gravity_FT_frame;
	try
	{
		tf_listener.transformVector(m_sensor_frame_id, gravity_, gravity_FT_frame);
	}

	catch(tf::TransformException &ex)
	{
		ROS_ERROR("Error transforming gravity vector...");
		ROS_ERROR("%s.", ex.what());
		return false;
	}

	geometry_msgs::WrenchStamped EndEffector_Weight;
	double EndEffectorMass = getEndEffectorMass();
	EndEffector_Weight.wrench.force.x = gravity_FT_frame.vector.x * EndEffectorMass;
	EndEffector_Weight.wrench.force.y = gravity_FT_frame.vector.y * EndEffectorMass;
	EndEffector_Weight.wrench.force.z = gravity_FT_frame.vector.z * EndEffectorMass;


	tf::StampedTransform transform;
	try
	{
		tf_listener.lookupTransform(m_sensor_frame_id, "/"+m_ArmSelect+"_arm_EndEffector_CenterMass", ros::Time(0), transform);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("Error looking up transform between FT sensor and end effector center of mass location");
		ROS_ERROR("%s.", ex.what());
		return false;
	}

	geometry_msgs::Vector3Stamped R;
	R.header.stamp = transform.stamp_;
	R.header.frame_id = transform.frame_id_;
	R.vector.x = transform.getOrigin().x();
	R.vector.y = transform.getOrigin().y();
	R.vector.z = transform.getOrigin().z();

	EndEffector_Weight.header = R.header;
	EndEffector_Weight.wrench.torque = cross(R.vector, EndEffector_Weight.wrench.force);

	// compensated FT in FT sensor frame
	geometry_msgs::Vector3Stamped FT_Fvec_comp;
	geometry_msgs::Vector3Stamped FT_Tvec_comp;
	FT_Fvec_comp.header.frame_id = EndEffector_Weight.header.frame_id;
	FT_Tvec_comp.header.frame_id = EndEffector_Weight.header.frame_id;
	FT_Fvec_comp.header.stamp = ros::Time();
	FT_Tvec_comp.header.stamp = ros::Time();

	FT_Fvec_comp.vector.x = FT_raw.wrench.force.x - EndEffector_Weight.wrench.force.x;
	FT_Fvec_comp.vector.y = FT_raw.wrench.force.y - EndEffector_Weight.wrench.force.y;
	FT_Fvec_comp.vector.z = FT_raw.wrench.force.z - EndEffector_Weight.wrench.force.z;

	FT_Tvec_comp.vector.x = FT_raw.wrench.torque.x - EndEffector_Weight.wrench.torque.x;
	FT_Tvec_comp.vector.y = FT_raw.wrench.torque.y - EndEffector_Weight.wrench.torque.y;
	FT_Tvec_comp.vector.z = FT_raw.wrench.torque.z - EndEffector_Weight.wrench.torque.z;

	// compensated FT vector in arm_base_link frame
//	geometry_msgs::Vector3Stamped FT_Fvec_comp_base;
//	geometry_msgs::Vector3Stamped FT_Tvec_comp_base;
//	try
//	{
//		tf_listener.transformVector("/arm_base_link", FT_Fvec_comp, FT_Fvec_comp_base);
//	}
//	catch(tf::TransformException &ex)
//	{
//		ROS_ERROR("Error transforming compensated FT force vector");
//		ROS_ERROR("%s.", ex.what());
//		return false;
//	}
//
//	try
//	{
//		tf_listener.transformVector("/arm_base_link", FT_Tvec_comp, FT_Tvec_comp_base);
//	}
//
//	catch(tf::TransformException &ex)
//	{
//		ROS_ERROR("Error transforming compensated FT torque vector");
//		ROS_ERROR("%s.", ex.what());
//		return false;
//	}

//	FT_comp.header = FT_Fvec_comp_base.header;
//	FT_comp.wrench.force = FT_Fvec_comp_base.vector;
//	FT_comp.wrench.torque = FT_Tvec_comp_base.vector;
//	FT_comp.header.stamp = FT_raw.header.stamp;
//	FT_comp.header.frame_id = FT_Fvec_comp_base.header.frame_id;

	// now expressed in the FT frame
	FT_comp.wrench.force = FT_Fvec_comp.vector;
	FT_comp.wrench.torque = FT_Tvec_comp.vector;
	FT_comp.header.stamp = FT_raw.header.stamp;
	FT_comp.header.frame_id = FT_raw.header.frame_id;

	return true;
}


bool ForceTorqueSensor::LPF(const geometry_msgs::WrenchStamped &FT_measurement,
		geometry_msgs::WrenchStamped &filtered_FT_measurement)
{
	static std::vector<Eigen::Vector3d> past_force(2,Eigen::Vector3d::Zero());
	static std::vector<Eigen::Vector3d> past_torque(2,Eigen::Vector3d::Zero());
	static std::vector<Eigen::Vector3d> past_filtered_force(2,Eigen::Vector3d::Zero());
	static std::vector<Eigen::Vector3d> past_filtered_torque(2,Eigen::Vector3d::Zero());

	// coefficients of the filter
	Eigen::Vector3d b;
	Eigen::Vector3d a;

	a = m_a;
	b = m_b;

	if(a==Vector3d::Zero()||b==Vector3d::Zero())
	{
		ROS_ERROR("LPF coefficients not set!");
		return false;
	}

	Eigen::Vector3d force(FT_measurement.wrench.force.x,
			FT_measurement.wrench.force.y,
			FT_measurement.wrench.force.z);

	Eigen::Vector3d torque(FT_measurement.wrench.torque.x,
			FT_measurement.wrench.torque.y,
			FT_measurement.wrench.torque.z);

	Eigen::Vector3d filtered_force;
	Eigen::Vector3d filtered_torque;

	filtered_force = b(0)*force + b(1)*past_force[0] + b(2)*past_force[1]
	                  - a(1)*past_filtered_force[0] - a(2) * past_filtered_force[1];

	filtered_torque = b(0)*torque + b(1)*past_torque[0] + b(2)*past_torque[1]
	                  - a(1)*past_filtered_torque[0] - a(2) * past_filtered_torque[1];


	past_force[1] = past_force[0];
	past_force[0] = force;

	past_torque[1] = past_torque[0];
	past_torque[0] = torque;

	past_filtered_force[1] = past_filtered_force[0];
	past_filtered_force[0] = filtered_force;

	past_filtered_torque[1] = past_filtered_torque[0];
	past_filtered_torque[0] = filtered_torque;


	filtered_FT_measurement.header = FT_measurement.header;
	filtered_FT_measurement.wrench.force.x = filtered_force(0);
	filtered_FT_measurement.wrench.force.y = filtered_force(1);
	filtered_FT_measurement.wrench.force.z = filtered_force(2);

	filtered_FT_measurement.wrench.torque.x = filtered_torque(0);
	filtered_FT_measurement.wrench.torque.y = filtered_torque(1);
	filtered_FT_measurement.wrench.torque.z = filtered_torque(2);

	return true;
}


geometry_msgs::Vector3 ForceTorqueSensor::cross(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2)
{
	geometry_msgs::Vector3 vec_out;

	vec_out.x = vec1.y*vec2.z - vec1.z*vec2.y;
	vec_out.y = vec1.z*vec2.x - vec1.x*vec2.z;
	vec_out.z = vec1.x*vec2.y - vec1.y*vec2.x;

	return vec_out;
}

bool ForceTorqueSensor::calibrateBias(unsigned int number_measurements, const geometry_msgs::Vector3Stamped &gravity)
{
	FT_CHECK_INITIALIZED();
	static bool first_calib = true;
	std::vector<double> FT_Bias(6,0.0);

	geometry_msgs::Wrench FT_raw;
	std::vector<double> previous_Bias = getBias();

	for(unsigned int i=0; i<number_measurements; i++)
	{
		ros::Duration(0.001).sleep();
		if(Get_FT(FT_raw))
		{

			// remove the bias
			FT_raw.force.x = FT_raw.force.x - previous_Bias[0];
			FT_raw.force.y = FT_raw.force.y - previous_Bias[1];
			FT_raw.force.z = FT_raw.force.z - previous_Bias[2];
			FT_raw.torque.x = FT_raw.torque.x - previous_Bias[3];
			FT_raw.torque.y = FT_raw.torque.y - previous_Bias[4];
			FT_raw.torque.z = FT_raw.torque.z - previous_Bias[5];


			geometry_msgs::WrenchStamped FT_raw_stamped;
			geometry_msgs::WrenchStamped FT_compensated_stamped;
			FT_raw_stamped.wrench = FT_raw;
			FT_raw_stamped.header.stamp = ros::Time::now();
			FT_raw_stamped.header.frame_id = m_sensor_frame_id;
			Compensate(FT_raw_stamped, gravity, FT_compensated_stamped);

			// transform to m_sensor_frame_id frame
			geometry_msgs::Vector3Stamped FT_Fvec;
			geometry_msgs::Vector3Stamped FT_Tvec;
			FT_Fvec.header.frame_id = FT_compensated_stamped.header.frame_id;
			FT_Tvec.header.frame_id = FT_compensated_stamped.header.frame_id;
			FT_Fvec.header.stamp = ros::Time();
			FT_Tvec.header.stamp = ros::Time();

			FT_Fvec.vector = FT_compensated_stamped.wrench.force;
			FT_Tvec.vector = FT_compensated_stamped.wrench.torque;

			geometry_msgs::Vector3Stamped FT_Fvec_FT_frame;
			geometry_msgs::Vector3Stamped FT_Tvec_FT_frame;
			try
			{
				tf_listener.transformVector(m_sensor_frame_id, FT_Fvec, FT_Fvec_FT_frame);
			}

			catch(tf::TransformException &ex)
			{
				ROS_ERROR("Error transforming FT force vector in computeBias()");
				ROS_ERROR("%s.", ex.what());
				return false;
			}

			try
			{
				tf_listener.transformVector(m_sensor_frame_id, FT_Tvec, FT_Tvec_FT_frame);
			}

			catch(tf::TransformException &ex)
			{
				ROS_ERROR("Error transforming FT torque vector in computeBias()");
				ROS_ERROR("%s.", ex.what());
				return false;
			}

			FT_Bias[0] -= FT_Fvec_FT_frame.vector.x;
			FT_Bias[1] -= FT_Fvec_FT_frame.vector.y;
			FT_Bias[2] -= FT_Fvec_FT_frame.vector.z;
			FT_Bias[3] -= FT_Tvec_FT_frame.vector.x;
			FT_Bias[4] -= FT_Tvec_FT_frame.vector.y;
			FT_Bias[5] -= FT_Tvec_FT_frame.vector.z;

		}

		else
		{
			return false;
		}
	}

	for(int i=0; i<6; i++) FT_Bias[i] = FT_Bias[i]/number_measurements;

	// running average with previous calibration of FT sensor
	std::vector<double> new_Bias(6,0.0);

	if(first_calib)
	{
		for(int i=0; i<6; i++)
		{
			new_Bias[i] = FT_Bias[i];
		}
		first_calib = false;
	}

	else
	{
		for(int i=0; i<6; i++)
		{
			new_Bias[i] = (FT_Bias[i] + previous_Bias[i])/2.0;
		}
	}

	(void)setBias(new_Bias);
	ROS_INFO("New %s arm FT sensor BIAS:", m_ArmSelect.c_str());
	std::cout << std::endl;
	for(int i=0; i<6; i++)
	{
		std::cout << std::setprecision(4) << std::setw(6) <<  new_Bias.at(i) << "  ";
	}
	std::cout << std::endl;

	return true;
}

