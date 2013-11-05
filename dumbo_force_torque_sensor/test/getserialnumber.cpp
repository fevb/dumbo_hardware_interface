/*
 *  getserialnumber
 *
 *  Created on: Aug 7, 2012
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


int main(int argc, char** argv)
{
	int ret;
	canHandle h;
	ros::init(argc, argv, "FT_get_SN");

	h = canOpenChannel(1, canWANT_EXCLUSIVE);
	ret = canSetBusParams(h, BAUD_250K, 0, 0, 0, 0, 0);

	if(ret<0) ROS_ERROR("Error setting bus params");

	ROS_INFO("Connected to bus.");

	canSetBusOutputControl(h, canDRIVER_NORMAL);
	canBusOn(h);
	char serial_char[8];
	std::string serial_number;
	get_Serial_Number(h, true, serial_char);
	serial_number.assign(serial_char, 8);

	ROS_INFO("Serial number: %s ", serial_number.c_str());

	return 0;
}
