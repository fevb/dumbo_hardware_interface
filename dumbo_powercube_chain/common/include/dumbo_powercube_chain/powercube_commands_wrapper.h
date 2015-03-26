/*
 * powercube_commands_wrapper
 *
 *  Created on: Aug 30, 2012
 *  Authors:   Francisco Viña 
 *            fevb <at> kth.se 
 */

/* Copyright (c) 2012, Francisco Vina, KTH
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

#ifndef __POWERCUBE_COMMANDS_WRAPPER_H_
#define __POWERCUBE_COMMANDS_WRAPPER_H_

#include <kvaser_canlib/canlib.h>
#include <iostream>
#include <vector>
extern "C"{
#include <dumbo_powercube_chain/powercube_defines.h>
}

// for compatibility
#define STATEID_MOD_POW_VOLT_ERR PC_STATE_POW_VOLT_ERROR
#define STATEID_MOD_ERROR PC_STATE_ERROR
#define STATEID_MOD_MOTION PC_STATE_MOTION
#define STATEID_MOD_HOME PC_STATE_HOME_OK

/* #define CONFIGID_MOD_SYNC_MOTION PC_CONFIGID_MOD_SYNC_MOTION */


void PCube_closeDevice(canHandle DeviceHandle);
  
// searches through CAN channels to find either the left or right arm
int PCube_openDevice(canHandle *DeviceHandle, int *CAN_Channel, std::string InitStr);

int PCube_resetAll(canHandle DeviceHandle);

int PCube_resetModule(canHandle DeviceHandle, int ModulID);

int PCube_setMinPos(canHandle DeviceHandle, int ModulID, float LowerLimit);

int PCube_setMaxPos(canHandle DeviceHandle, int ModulID, float UpperLimit);
  
int PCube_haltAll(canHandle DeviceHandle);

int PCube_haltModule(canHandle DeviceHandle, int ModulID);
  
int PCube_setMaxVel(canHandle DeviceHandle, int ModuleID, float maxVelocity);

int PCube_setMaxAcc(canHandle DeviceHandle, int ModuleID, float maxAcceleration);

int PCube_getMaxCurrent(canHandle DeviceHandle, int ModuleID, float *maxCurrent);

int PCube_setMaxCurrent(canHandle DeviceHandle, int ModuleID, float maxCurrent);

int PCube_getConfig(canHandle DeviceHandle, int ModuleID, unsigned long int *confword);

int PCube_setConfig(canHandle DeviceHandle, int ModuleID, unsigned long int confword);

int PCube_getStateDioPos(canHandle DeviceHandle, int ModuleID, unsigned long int *state,
			 unsigned char *dio, float *position);

int PCube_getSerialNumber(canHandle DeviceHandle, int ModuleID, unsigned long int *SerialNumber);

int PCube_homeModule(canHandle DeviceHandle, int ModuleID);
  
int PCube_getModuleState(canHandle DeviceHandle, int ModuleID, unsigned long int *state);

int PCube_getModulePos(canHandle DeviceHandle, int ModuleID, float *pos);


int PCube_movePos(canHandle DeviceHandle, const std::vector<int> &ModuleIDs, const std::vector<float> &target_pos, std::vector<float> &pos);

int PCube_moveModulePos(canHandle DeviceHandle, int ModuleID, const float target_pos, unsigned long int *ShortState, unsigned char *dio, float *pos);

int PCube_moveStepExtended(canHandle DeviceHandle, int ModuleID, float target_pos, unsigned short target_time, unsigned long int *ShortState, unsigned char *dio, float *pos);

// same as previous command but doesn't wait for the reply from the module
int PCube_moveStepExtendedNoRead(canHandle DeviceHandle, int ModuleID, float target_pos, unsigned short target_time);

// to be executed after a motion command on a module to catch the feedback msg from the module
int PCube_readState(canHandle DeviceHandle, int ModuleID, unsigned long int *ShortState, unsigned char *dio, float *pos, bool wait_for_response=true);

int PCube_moveModuleVel(canHandle DeviceHandle, int ModuleID, float target_vel, unsigned long int *ShortState, unsigned char *dio, float *pos);

int PCube_startMotionAll(canHandle DeviceHandle);


std::vector<float> PCube_correctAngles(const std::vector<float> &angles);

float PCube_correctAngle(int ModuleID, const float angle);

std::vector<float> PCube_correctVel(const std::vector<float> &vel);

float PCube_correctVel(int ModuleID, const float vel);

unsigned long int PCube_Process_ShortState(char ShortState);
  

#endif
