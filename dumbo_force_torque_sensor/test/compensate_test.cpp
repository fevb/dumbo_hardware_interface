/*
 *  compensate_test
 *
 *  Created on: Aug 12, 2012
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
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <math.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "compensate_test");
	ros::NodeHandle n;
	n = ros::NodeHandle("~");
	geometry_msgs::WrenchStamped FT_raw;
	std::string ArmSelect = "left";
	std::string Serial_Number = "FT";
	std::vector<double> Bias(6,0);
	double EndEffectorMass = 1.69;

	ForceTorqueSensor FT_sensor(Serial_Number, ArmSelect, Bias, EndEffectorMass);

	FT_raw.header.frame_id = "left_arm_FT_sensor";
	FT_raw.header.stamp = ros::Time::now();
	FT_raw.wrench.force.x = 0.0;
	FT_raw.wrench.force.y = -1.69*9.81;
	FT_raw.wrench.force.z = 0.0;
	FT_raw.wrench.torque.x = (0.296-0.218)*1.69*9.81;
	FT_raw.wrench.torque.y = 0.0;
	FT_raw.wrench.torque.z = 0.0;

	geometry_msgs::WrenchStamped FT_comp;
	ros::Publisher topicPub_FT_raw_ = n.advertise<geometry_msgs::WrenchStamped> ("FT_raw", 1);
	ros::Publisher topicPub_FT_compensated_ = n.advertise<geometry_msgs::WrenchStamped> ("FT_compensated", 1);

	geometry_msgs::Vector3Stamped gravity;

	gravity.header.stamp = ros::Time::now();
	gravity.header.frame_id = "/arm_base_link";
	gravity.vector.x = 0.0;
	gravity.vector.x = 0.0;
	gravity.vector.x = -9.81;

	ros::Rate loop_rate(100);
	ros::Time t_start = ros::Time::now();



	while (n.ok())
	{
		ros::Duration delta_t = t_start - ros::Time::now();
		FT_raw.wrench.force.y = -1.69*9.81 + 0.5*sin(2*3.1416*5*delta_t.toSec());
		FT_raw.header.stamp = ros::Time::now();
		FT_sensor.Compensate(FT_raw, gravity, FT_comp);
		topicPub_FT_raw_.publish(FT_raw);
		topicPub_FT_compensated_.publish(FT_comp);
		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
