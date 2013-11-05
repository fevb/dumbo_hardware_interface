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
#include <tf/transform_listener.h>
#include <dumbo_srvs/CalibrateFT.h>
#include <sensor_msgs/Imu.h>

class ForceTorqueNode
{

public:
	/// create a handle for this node, initialize node
	ros::NodeHandle n_;

	// subscribe to accelerometer readings
	ros::Subscriber topicSub_Accelerometer_;

	/// declaration of topics to publish
	ros::Publisher topicPub_FT_raw_;
	ros::Publisher topicPub_FT_compensated_;
	ros::Publisher topicPub_FT_compensated_filtered;

	/// declaration of service servers
	ros::ServiceServer srvServer_Connect_;
	ros::ServiceServer srvServer_Disconnect_;
	ros::ServiceServer srvServer_Calibrate_;

	ForceTorqueSensor *m_FT_Sensor;

	///Constructor
	ForceTorqueNode()
	{
		n_ = ros::NodeHandle("~");

		// subscribe to accelerometer topic
		topicSub_Accelerometer_ = n_.subscribe("imu", 1, &ForceTorqueNode::topicCallback_AccelerometerReading, this);

		/// implementation of topics to publish
		topicPub_FT_raw_ = n_.advertise<geometry_msgs::WrenchStamped> ("FT_raw", 1);
		topicPub_FT_compensated_ = n_.advertise<geometry_msgs::WrenchStamped> ("FT_compensated", 1);
		topicPub_FT_compensated_filtered = n_.advertise<geometry_msgs::WrenchStamped> ("FT_compensated_filtered", 1);

		/// implementation of service servers
		srvServer_Connect_ = n_.advertiseService("connect", &ForceTorqueNode::srvCallback_Connect, this);
		srvServer_Disconnect_ = n_.advertiseService("disconnect", &ForceTorqueNode::srvCallback_Disconnect, this);
		srvServer_Calibrate_  = n_.advertiseService("calibrate", &ForceTorqueNode::srvCallback_Calibrate, this);

		if(GetROSParams())
		{
			m_FT_Sensor = new ForceTorqueSensor(m_serial_number, m_arm_select, m_bias, m_end_effector_mass);
			m_FT_Sensor->SetLPFCoeff(m_a, m_b);
		}

		m_received_imu = false;
	}

	~ForceTorqueNode()
	{
		delete m_FT_Sensor;
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

		/// Get max accelerations
		XmlRpc::XmlRpcValue BiasXmlRpc;
		std::vector<double> Bias;
		if (n_.hasParam("bias"))
		{
			n_.getParam("bias", BiasXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter bias not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to the MaxAccelerations
		Bias.resize(BiasXmlRpc.size());
		for (int i = 0; i < BiasXmlRpc.size(); i++)
		{
			Bias[i] = (double)BiasXmlRpc[i];
		}

		if(Bias.size()!=6)
		{
			ROS_ERROR("Wrong FT bias size, shutting down node");
			n_.shutdown();
			return false;
		}

		double EndEffectorMass;
		if (n_.hasParam("end_effector_mass"))
		{
			n_.getParam("end_effector_mass", EndEffectorMass);
		}

		else
		{
			ROS_ERROR("Parameter end_effector_mass not available");
			n_.shutdown();
			return false;
		}

		/// Get b coeff of LPF
		XmlRpc::XmlRpcValue b_XmlRpc;
		std::vector<double> b_;
		if (n_.hasParam("b"))
		{
			n_.getParam("b", b_XmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter b not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(b_XmlRpc.size()!=3)
		{
			ROS_ERROR("Wrong b parameter size.");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to b_
		b_.resize(b_XmlRpc.size());
		for (int i = 0; i < b_XmlRpc.size(); i++)
		{
			b_[i] = (double)b_XmlRpc[i];
		}


		/// Get a coeff of LPF
		XmlRpc::XmlRpcValue a_XmlRpc;
		std::vector<double> a_;
		if (n_.hasParam("a"))
		{
			n_.getParam("a", a_XmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter a not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(a_XmlRpc.size()!=3)
		{
			ROS_ERROR("Wrong a parameter size.");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to a_
		a_.resize(a_XmlRpc.size());
		for (int i = 0; i < a_XmlRpc.size(); i++)
		{
			a_[i] = (double)a_XmlRpc[i];
		}

		m_serial_number = SerialNumber;
		m_arm_select = ArmSelect;
		m_bias = Bias;
		m_end_effector_mass = EndEffectorMass;
		m_b << b_[0], b_[1], b_[2] ;
		m_a << a_[0], a_[1], a_[2] ;


		return true;
	}

	void topicCallback_AccelerometerReading(const sensor_msgs::Imu::ConstPtr &msg)
	{
		ROS_DEBUG("In accelerometer read callback");

		m_gravity.header = msg->header;
		m_gravity.vector = msg->linear_acceleration;

		m_received_imu = true;
	}

	void PublishFT()
	{
		geometry_msgs::Wrench FT_raw;
		geometry_msgs::WrenchStamped FT_raw_stamped;
		geometry_msgs::WrenchStamped FT_compensated_stamped;
		geometry_msgs::WrenchStamped FT_compensated_filtered;

		if(m_FT_Sensor->isInitialized())
		{
			if(m_FT_Sensor->Get_FT(FT_raw))
			{
				FT_raw_stamped.wrench = FT_raw;
				FT_raw_stamped.header.stamp = ros::Time::now();
				FT_raw_stamped.header.frame_id = "/" + m_arm_select + "_arm_FT_sensor";
				topicPub_FT_raw_.publish(FT_raw_stamped);

				if(!m_received_imu)
				{
					ROS_ERROR("Haven't received accelerometer reading, cannot compensate FT measurement...");
					return;
				}

				if((ros::Time::now()-m_gravity.header.stamp).toSec()>0.2)
				{
					ROS_ERROR("Accelerometer reading too old, cannot compensate FT measurement...");
					return;
				}

				if(m_FT_Sensor->Compensate(FT_raw_stamped, m_gravity, FT_compensated_stamped))
				{
					topicPub_FT_compensated_.publish(FT_compensated_stamped);

					if(m_FT_Sensor->LPF(FT_compensated_stamped, FT_compensated_filtered))
					{
						topicPub_FT_compensated_filtered.publish(FT_compensated_filtered);
					}
				}
			}
		}

	}


	bool srvCallback_Connect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		if(m_FT_Sensor->Init())
		{
			res.success.data = true;
			return true;
		}

		ROS_ERROR("Error initializing FT sensor");
		res.success.data = false;
		res.error_message.data = "error initialization";
		return false;
	}

	bool srvCallback_Disconnect(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		m_FT_Sensor->Disconnect();
		ROS_INFO("FT sensor disconnected.");
		res.success.data = true;
		return true;
	}

	bool srvCallback_Calibrate(dumbo_srvs::CalibrateFT::Request &req, dumbo_srvs::CalibrateFT::Response &res)
	{
		if(m_FT_Sensor->calibrateBias(req.num_measurements, m_gravity))
		{
			res.success = true;
			return true;
		}

		else
		{
			res.success = false;
			return false;
		}
	}

private:
	std::string m_serial_number;
	std::string m_arm_select;
	std::vector<double> m_bias;
	double m_end_effector_mass;
	Eigen::Vector3d m_b;
	Eigen::Vector3d m_a;

	bool m_received_imu;

	geometry_msgs::Vector3Stamped m_gravity;


};
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "force_torque");
	ForceTorqueNode FT_node;

	double loop_frequency;
	double read_frequency;
	FT_node.n_.param("loop_frequency", loop_frequency, 500.0);
	FT_node.n_.param("read_frequency", read_frequency, 400.0);

	if(loop_frequency<=0.0)
		loop_frequency = 500.0;

	if(read_frequency<=0.0)
			read_frequency = 400.0;

	ros::Time t_last_read = ros::Time::now();




	/// main loop
	ros::Rate loop_rate(loop_frequency); // Hz
	while (FT_node.n_.ok())
	{
//		if((ros::Time::now()-t_last_read).toSec() > 1.0/read_frequency)
//		{
			FT_node.PublishFT();
//			t_last_read = ros::Time::now();
//		}

		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
