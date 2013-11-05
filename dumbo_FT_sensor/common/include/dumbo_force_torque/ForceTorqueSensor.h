/*
 *  ForceTorqueSensor.h
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

#ifndef FORCETORQUESENSOR_H_
#define FORCETORQUESENSOR_H_

#include <string>
#include <kvaser_canlib/canlib.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pthread.h>
#include <dumbo_force_torque/ft_sensor_function.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>


class ForceTorqueSensor {

	tf::TransformListener tf_listener;
public:
	ForceTorqueSensor(std::string Serial_Number,
			std::string ArmSelect,
			std::vector<double> Bias,
			double EndEffectorMass);

	virtual ~ForceTorqueSensor();
	bool Init();
	void Disconnect();

	bool isInitialized()
	{
		return (m_Initialized);
	}

	bool Get_FT(geometry_msgs::Wrench &FT_raw);

	std::vector<double> getBias()
	{
		return m_Bias;
	}

	void setBias(const std::vector<double> &Bias)
	{
		if(Bias.size()==6)
		{
			m_Bias = Bias;
		}

		else
		{
			ROS_ERROR("Incorrect Bias size FT sensor.");
		}

	}

	bool SetLPFCoeff(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
	void GetLPFCoeff(Eigen::Vector3d &a, Eigen::Vector3d &b);

	double getEndEffectorMass()
	{
		return m_EndEffectorMass;
	}

	void setEndEffectorMass(double EndEffectorMass)
	{
		m_EndEffectorMass = EndEffectorMass;
	}

	// converts gravity vector to the correct frame
	bool Compensate(const geometry_msgs::WrenchStamped &FT_measurement_raw,
			const geometry_msgs::Vector3Stamped &gravity,
			geometry_msgs::WrenchStamped &FT_measurement_comp);

	// Butterworth 2nd order Low Pass Filter
	bool LPF(const geometry_msgs::WrenchStamped &FT_measurement,
			geometry_msgs::WrenchStamped &filtered_FT_measurement);

	geometry_msgs::Vector3 cross(const geometry_msgs::Vector3 &vec1,
			const geometry_msgs::Vector3 &vec2);

	bool calibrateBias(unsigned int number_measurements, const geometry_msgs::Vector3Stamped &gravity);

protected:
	pthread_mutex_t m_CAN_mutex;

	std::string m_SerialNumber;
	std::string m_ArmSelect;
	std::string m_sensor_frame_id;
	canHandle m_DeviceHandle;
	int m_CAN_Channel;
	bool m_Initialized;
	bool m_CANDeviceOpened;

	float m_Calibration_Matrix[6][6];
	std::vector<double> m_Bias;

	tf::TransformListener *m_tf_listener;
	double m_EndEffectorMass;

	// LPF coefficients
	Eigen::Vector3d m_b;
	Eigen::Vector3d m_a;

};

#endif /* FORCETORQUESENSOR_H_ */
