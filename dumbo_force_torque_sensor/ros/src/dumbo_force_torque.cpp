/*
 *  dumbo_force_torque
 *
 *  Created on: Aug 30, 2012
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
#include <dumbo_force_torque_sensor/ForceTorqueSensor.h>
#include <cob_srvs/Trigger.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

class ForceTorqueNode
{

public:
	/// create a handle for this node, initialize node
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_ft_raw_;

	/// declaration of service servers
	ros::ServiceServer srvServer_Connect_;
	ros::ServiceServer srvServer_Disconnect_;

	ForceTorqueSensor *m_ft_sensor;

	///Constructor
	ForceTorqueNode()
	{
		n_ = ros::NodeHandle("~");

		/// implementation of topics to publish
		topicPub_ft_raw_ = n_.advertise<geometry_msgs::WrenchStamped> ("ft_raw", 1);

		/// implementation of service servers
		srvServer_Connect_ = n_.advertiseService("connect", &ForceTorqueNode::srvCallback_Connect, this);
		srvServer_Disconnect_ = n_.advertiseService("disconnect", &ForceTorqueNode::srvCallback_Disconnect, this);

		if(GetROSParams())
		{
			m_ft_sensor = new ForceTorqueSensor(m_serial_number, m_arm_select);
		}
	}

	~ForceTorqueNode()
	{
		delete m_ft_sensor;
	}

	bool GetROSParams()
	{
		std::string SerialNumber;
		if (n_.hasParam("serial_number"))
		{
			n_.getParam("serial_number", SerialNumber);
		}

		else
		{
			ROS_ERROR("Parameter SerialNumber not available");
			n_.shutdown();
			return false;
		}

		std::string ArmSelect;
		if (n_.hasParam("arm_select"))
		{
			n_.getParam("arm_select", ArmSelect);
		}

		else
		{
			ROS_ERROR("Parameter ArmSelect not available");
			n_.shutdown();
			return false;
		}


		m_serial_number = SerialNumber;
		m_arm_select = ArmSelect;

		return true;
	}

	void Publish_ft()
	{
		geometry_msgs::Wrench ft_raw;
		geometry_msgs::WrenchStamped ft_raw_stamped;

		if(m_ft_sensor->isInitialized())
		{
            if(m_ft_sensor->getFT(ft_raw))
			{
				ft_raw_stamped.wrench = ft_raw;
				ft_raw_stamped.header.stamp = ros::Time::now();
				ft_raw_stamped.header.frame_id = "/" + m_arm_select + "_arm_ft_sensor";
				topicPub_ft_raw_.publish(ft_raw_stamped);
			}
		}

	}


	bool srvCallback_Connect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		if(m_ft_sensor->Init())
		{
			res.success.data = true;
			return true;
		}

		ROS_ERROR("Error initializing F/T sensor");
		res.success.data = false;
		res.error_message.data = "error initialization";
		return false;
	}

	bool srvCallback_Disconnect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		m_ft_sensor->Disconnect();
		ROS_INFO("F/T sensor disconnected.");
		res.success.data = true;
		return true;
	}


private:
	std::string m_serial_number;
	std::string m_arm_select;

};
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "force_torque");
	ForceTorqueNode ft_sensor_node;

	double loop_frequency;
	ft_sensor_node.n_.param("loop_frequency", loop_frequency, 500.0);

	if(loop_frequency<=0.0)
		loop_frequency = 500.0;


	/// main loop
	ros::Rate loop_rate(loop_frequency); // Hz
	while (ft_sensor_node.n_.ok())
	{
		ft_sensor_node.Publish_ft();

		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
