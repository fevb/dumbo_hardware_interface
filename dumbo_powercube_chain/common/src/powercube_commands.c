/*
 * powercube_commands
 *
 *  Created on: 2005
 *  Authors:   Christian Smith
 *            ccs <at> kth.se 
 */

/* Copyright (c) 2005, Christian Smith, KTH
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

/* Written by Christian Smith, Oct 2005*/
/*
  Library of powercube commands.
*/
// gcc -Wall -O2  -D_REENTRANT -I../../include -L .. -lcanlib -lpthread powercube_tester.c powercube_commands.c -o powercube_tester

#include <string.h>
#include <kvaser_canlib/canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <dumbo_powercube_chain/powercube_defines.h>
#include <dumbo_powercube_chain/powercube_commands.h>
#include <pthread.h>

#ifndef PI
#define PI (3.141592653)
#endif

#ifndef SELECT_LEFT_ARM
#define SELECT_LEFT_ARM 0
#endif

#ifndef SELECT_RIGHT_ARM
#define SELECT_RIGHT_ARM	    1
#endif

static int kill_watchdog = 1;

unsigned int can_flags = canMSG_STD; // + other needed flags? see canstat.h

int pc_get_data_type(unsigned char commandID, unsigned char paramID){
  printf("getDtype: cID: %#02X, pID: %#02X\n",commandID, paramID);
  switch (commandID){
  case PC_COMMAND_RESET:
  case PC_COMMAND_HOME:
  case PC_COMMAND_HALT:
  case PC_COMMAND_RECALC_PID_PARAM:
    return -1;
    break;
  case PC_COMMAND_SET_EXTENDED:
  case PC_COMMAND_GET_EXTENDED:
    switch (paramID){
    case PC_PARA_DEF_HOME_OFFSET:
    case PC_PARA_DEF_GEAR_RATIO:
    case PC_PARA_DEF_LIN_RATIO:
    case PC_PARA_DEF_MIN_POS:
    case PC_PARA_DEF_MAX_POS:
    case PC_PARA_DEF_MAX_DELTA_POS:
    case PC_PARA_DEF_MAX_DELTA_VEL:
    case PC_PARA_DEF_TORQUE_RATIO:
    case PC_PARA_DEF_CUR_RATIO:
    case PC_PARA_DEF_MIN_VEL:
    case PC_PARA_DEF_MAX_VEL:
    case PC_PARA_DEF_MIN_ACC:
    case PC_PARA_DEF_MAX_ACC:
    case PC_PARA_DEF_MIN_CUR:
    case PC_PARA_DEF_MAX_CUR:
    case PC_PARA_DEF_HOME_VEL:
    case PC_PARA_DEF_HOME_ACC:
    case PC_PARA_INC_RATIO:
    case PC_PARA_ACT_POS:
    case PC_PARA_IPOL_POS:
    case PC_PARA_DELTA_POS:
    case PC_PARA_MAX_DELTA_POS:
    case PC_PARA_ACT_VEL:
    case PC_PARA_IPOL_VEL:
    case PC_PARA_MIN_POS:
    case PC_PARA_MAX_POS:
    case PC_PARA_MAX_VEL:
    case PC_PARA_MAX_ACC:
    case PC_PARA_MAX_CUR:
    case PC_PARA_CUR:
    case PC_PARA_TARGET_POS:
    case PC_PARA_TARGET_VEL:
    case PC_PARA_TARGET_ACC:
    case PC_PARA_HOME_OFFSET:
      return PC_DATA_TYPE_FLOAT;
      break;

    case PC_PARA_DEF_CUBE_SERIAL:
    case PC_PARA_DEF_CONFIG:
    case PC_PARA_DEF_PULSES_PER_TURN:
    case PC_PARA_POS_COUNT:
    case PC_PARA_REF_POS_COUNT:
    case PC_PARA_HOME_OFFSET_INC:
    case PC_PARA_HOME_TO_ZERO_INC:
      return PC_DATA_TYPE_INT32;
      break;
    
    case PC_PARA_DEF_CUBE_VERSION:
    case PC_PARA_DEF_SERVICE_INTERVAL:
    case PC_PARA_DEF_BRAKE_TIME_OUT:    
      return PC_DATA_TYPE_UINT16;
      break;
      
    case PC_PARA_DEF_ADDRESS:
    case PC_PARA_DEF_PRIM_BAUD:
    case PC_PARA_DEF_SCND_BAUD:
    case PC_PARA_MOVE_MODE:
      return PC_DATA_TYPE_UINT8;
      break;

    case PC_PARA_DIO_SETUP:
    case PC_PARA_CUBE_STATE:
    case PC_PARA_TARGET_POS_INC:
    case PC_PARA_TARGET_VEL_INC:
    case PC_PARA_TARGET_ACC_INC:
    case PC_PARA_STEP_INC:
    case PC_PARA_SETUP:
    case PC_PARA_CONFIG:
      return PC_DATA_TYPE_UINT32;
      break;

    case PC_PARA_RAW_CUR:
    case PC_PARA_ACT_C0:
    case PC_PARA_ACT_DAMP:
    case PC_PARA_ACT_A0:
    case PC_PARA_DEF_C0:
    case PC_PARA_DEF_DAMP:
    case PC_PARA_DEF_A0:
      return PC_DATA_TYPE_INT16;
      break;
    }
    return -1;
    break;
  case PC_COMMAND_SET_MOTION:
    switch (paramID){
      // Floats
    case PC_MOTION_FRAMP_MODE:
    case PC_MOTION_FVEL_MODE:
    case PC_MOTION_FCUR_MODE:
    case PC_MOTION_FRAMP_ACK:
    case PC_MOTION_FVEL_ACK:
    case PC_MOTION_FCUR_ACK:
      return PC_DATA_TYPE_FLOAT;
      break;

    case PC_MOTION_FSTEP_MODE:
    case PC_MOTION_FSTEP_ACK:
      return PC_DATA_TYPE_FLOAT_UINT16;
      break;
    case PC_MOTION_IRAMP_MODE:
    case PC_MOTION_IVEL_MODE:
    case PC_MOTION_IRAMP_ACK:
    case PC_MOTION_IVEL_ACK:
      return PC_DATA_TYPE_INT32;
      break;

    case PC_MOTION_ICUR_MODE:
    case PC_MOTION_ICUR_ACK:
      return PC_DATA_TYPE_INT16;
      break;
      
    case PC_MOTION_ISTEP_MODE:
    case PC_MOTION_ISTEP_ACK:
     return PC_DATA_TYPE_INT32_UINT16;
     break;
    }
    return -1;
    break;
  case PC_COMMAND_SET_I_STEP:
    return -1;
  }
  return -1;  
}


/* 
   The following commands can be used to set the char[8] data message's 
   byte 2-7 to contain data of different types. The returned value can
   be used as dlc
*/
unsigned int set_data_int(unsigned char msg[8], long int *val){
  memcpy(&(msg[2]), val, 4);
  return 6;
}

unsigned int set_data_uint32(unsigned char msg[8], unsigned long int *val){
  memcpy(&(msg[2]), val, 4);
  return 6;
}

unsigned int set_data_int16(unsigned char msg[8], short int *val){
  memcpy(&(msg[2]), val, 2);
  return 4;
}

unsigned int set_data_uint16(unsigned char msg[8], unsigned short int *val){
  memcpy(&(msg[2]), val, 2);
  return 4;
}

unsigned int set_data_float(unsigned char msg[8], float *val){
  memcpy(&(msg[2]), val, 4);
  return 6;
}

unsigned int set_data_float_uint16(unsigned char msg[8], float *fval, unsigned short int *ival){
  memcpy(&(msg[2]), fval, 4);
  memcpy(&(msg[6]), ival, 2);
  return 8;
}

unsigned int set_data_int_uint16(unsigned char msg[8], long int *ival32, unsigned short int *ival16){
  memcpy(&(msg[2]), ival32, 4);
  memcpy(&(msg[6]), ival16, 2);
  return 8;
}


/* 
   The following commands can be used to get the char[8] data message's 
   byte 2-7 and store the results as different types
*/
void get_data_int(unsigned char msg[8], long int *val){
  memcpy(val, &(msg[2]), 4);
}

void get_data_uint32(unsigned char msg[8], unsigned long int *val){
  memcpy(val, &(msg[2]), 4);
}

void get_data_int16(unsigned char msg[8], short int *val){
  memcpy(val, &(msg[2]), 2);
}

void get_data_uint16(unsigned char msg[8], unsigned short int *val){
  memcpy(val, &(msg[2]), 2);
}

void get_data_float(unsigned char msg[8], float *val){
  memcpy(val, &(msg[2]), 4);
}

void get_data_float_uint16(unsigned char msg[8], float *fval, unsigned short int *ival){
  memcpy(fval, &(msg[2]), 4);
  memcpy(ival, &(msg[6]), 2);
}

void get_data_int_uint16(unsigned char msg[8], long int *ival32, unsigned short int *ival16){
  memcpy(ival32, &(msg[2]), 4);
  memcpy(ival16, &(msg[6]), 2);
}

/*
  Sends 'halt' command to all modules on bus h
*/
void pc_halt_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_HALT; 
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'reset' command to all modules on bus h
*/
void pc_reset_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_RESET; 
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'home' command to all modules on bus h
*/
void pc_home_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_HOME; 
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'set baudrate' command to all modules on bus h
  Baudrate must be one of 
  {PC_BAUDRATE_250K, PC_BAUDRATE_500K, PC_BAUDRATE_1M}
*/
void pc_set_baud_all_on_bus(canHandle h, unsigned char baudrate){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_CHANGE_BAUDRATE; 
  msg[1] = baudrate;
  canWriteWait(h, PC_CANID_CMDALL, msg, 2, can_flags, 1000);
}

/*
  Sends 'save pos' command to all modules on bus h
*/
void pc_save_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_SAVE_POSITION; 
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'watchdog refresh' command to all modules on bus h
*/
void pc_watch_refresh_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_WATCHDOG_REFRESH;
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'synchronize' command to all modules on bus h
*/
void pc_sync_all_on_bus(canHandle h){
  unsigned char msg[8];
  msg[0] = PC_ALL_COMMAND_SYNC_MOTION;
  canWriteWait(h, PC_CANID_CMDALL, msg, 1, can_flags, -1);
}

/*
  Sends 'halt' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_halt_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4){
  pc_halt_all_on_bus(h1);
  pc_halt_all_on_bus(h2);
  pc_halt_all_on_bus(h3);
  pc_halt_all_on_bus(h4);
}

/*
  Sends 'reset' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_reset_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4){
  pc_reset_all_on_bus(h1);
  pc_reset_all_on_bus(h2);
  pc_reset_all_on_bus(h3);
  pc_reset_all_on_bus(h4);
}


/*
  Sends 'watchdog refresh' command to all modules on all 
  four buses (h1, h2, h3, and h4)
*/
void pc_watch_refresh_all(canHandle h1, canHandle h2, 
			  canHandle h3, canHandle h4){
  pc_watch_refresh_all_on_bus(h1);
  pc_watch_refresh_all_on_bus(h2);
  pc_watch_refresh_all_on_bus(h3);
  pc_watch_refresh_all_on_bus(h4);
}


/*
  Sends 'set baudrate' command to all modules on all 
  four buses (h1, h2, h3, and h4)
  Baudrate must be one of 
  {PC_BAUDRATE_250K, PC_BAUDRATE_500K, PC_BAUDRATE_1M}
*/
void pc_set_baud_all(canHandle h1, canHandle h2, 
		     canHandle h3, canHandle h4, 
		     unsigned char baudrate){
  pc_set_baud_all_on_bus(h1,baudrate);
  pc_set_baud_all_on_bus(h2,baudrate);
  pc_set_baud_all_on_bus(h3,baudrate);
  pc_set_baud_all_on_bus(h4,baudrate);
}


/*
  Sends 'home' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_home_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4){
  pc_home_all_on_bus(h1);
  pc_home_all_on_bus(h2);
  pc_home_all_on_bus(h3);
  pc_home_all_on_bus(h4);
}

/*
  Resets a single module
*/
void pc_reset_module(struct module _mod){
  unsigned char msg[8];
  msg[0] = PC_COMMAND_RESET; 
  canWriteWait(_mod.handle, (PC_CANID_CMDPUT + _mod.canID), msg, 1, can_flags, -1);
}


/*
  Homes a single module
*/
void pc_home_module(struct module _mod){
  unsigned char msg[8];
  msg[0] = PC_COMMAND_HOME; 
  canWriteWait(_mod.handle, (PC_CANID_CMDPUT + _mod.canID), msg, 1, can_flags, -1);
}

/*
  Halts a single module
*/
void pc_halt_module(struct module _mod){
  unsigned char msg[8];
  msg[0] = PC_COMMAND_HALT; 
  canWriteWait(_mod.handle, (PC_CANID_CMDPUT + _mod.canID), msg, 1, can_flags, -1);
  //printf("\n id = %ld",_mod.canID);
}

/*
  Sends a command to a single module.
*/
void pc_send_command_to_module(struct module _mod, unsigned char msg[8], unsigned int dlc){
  canWriteWait(_mod.handle, (PC_CANID_CMDPUT + _mod.canID), msg, dlc, can_flags, -1);
}

/*
  Sends a command to a single module, without waiting.
*/
void pc_send_command_to_module_nowait(struct module _mod, unsigned char msg[8], unsigned int dlc){
  canWrite(_mod.handle, (PC_CANID_CMDPUT + _mod.canID), msg, dlc, can_flags);
}


/*
  Returns the status word of a single module
*/
unsigned long int pc_get_status(struct module _mod){
  unsigned char _msg[8];
  unsigned long int _ret;
  _msg[0] = PC_COMMAND_GET_EXTENDED;
  _msg[1] = PC_PARA_CUBE_STATE;
  pc_request_value_from_module(_mod,_msg);
  if(pc_listen_for_response(_mod.handle,_msg)<0){
    return 0;
  }
  get_data_uint32(_msg,&_ret);
  return _ret;
}

/*
  Prints the module state given a module state word
 */
void pc_print_module_state(unsigned long int _cube_state){
  if(_cube_state & PC_STATE_ERROR)
    printf("GENERAL ERROR! (bit 0)\n");
  if(_cube_state & PC_STATE_HOME_OK)
    printf("HOMING SUCCESFUL! (bit 1)\n");
  if(_cube_state & PC_STATE_HALTED)
    printf("MODULE HALTED! (bit 2)\n");
  if(_cube_state & PC_STATE_POWERFAULT)
    printf("POWER FAULT! (bit 3)\n");
  
  if(_cube_state & PC_STATE_TOW_ERROR)
    printf("TOW ERROR! (bit 4)\n");
  if(_cube_state & PC_STATE_COMM_ERROR)
    printf("COMM ERROR! (bit 5)\n");
  if(_cube_state & PC_STATE_SWR)
    printf("HOME SWITCH (bit 6)\n");
  if(_cube_state & PC_STATE_SW1)
    printf("LIMIT 1 (bit 7)\n");
	
  if(_cube_state & PC_STATE_SW2)
    printf("LIMIT 2 (bit 8)\n");
  if(_cube_state & PC_STATE_BRAKE_ACTIVE)
    printf("BRAKE ACTIVE (bit 9)\n");
  if(_cube_state & PC_STATE_CUR_LIMIT)
    printf("CURRENT LIMIT REACHED (bit 10)\n");
  if(_cube_state & PC_STATE_MOTION)
    printf("IN MOTION (bit 11)\n");
  
  if(_cube_state & PC_STATE_RAMP_ACC)
    printf("RAMP ACCELERATE (bit 12)\n");
  if(_cube_state & PC_STATE_RAMP_STEADY)
    printf("RAMP STEADY (bit 13)\n");
  if(_cube_state & PC_STATE_RAMP_DEC)
    printf("RAMP DECELERATE (bit 14)\n");
  if(_cube_state & PC_STATE_RAMP_END)
    printf("RAMP ENDED (bit 15)\n");

  if(_cube_state & PC_STATE_IN_PROGRESS)
    printf("STEP IN PROGRESS (bit 16)\n");
  if(_cube_state & PC_STATE_FULLBUFFER)
    printf("STEP BUFFER FULL (bit 17)\n");
  if(_cube_state & PC_STATE_POW_VOLT_ERROR)
    printf("VOLTAGE ERROR! (bit 18)\n");
  if(_cube_state & PC_STATE_POW_FET_TEMP)
    printf("TRANSISTOR OVERHEAT! (bit 19)\n");
  
  if(_cube_state & PC_STATE_POW_WDG_TEMP)
    printf("MOTOR OVERHEAT! (bit 20)\n");
  if(_cube_state & PC_STATE_POW_SHORT_CURR)
    printf("SHORT CIRCUIT! (bit 21)\n");
  if(_cube_state & PC_STATE_POW_HALLERR)
    printf("HALL EFFECT SENSOR ERROR! (bit 22)\n");
  if(_cube_state & PC_STATE_POW_INTEGRAL_ERR)
    printf("INTEGRAL ERROR! (bit 23)\n");
  
  if(_cube_state & PC_STATE_CPU_OVERLOAD)
    printf("CPU OVERLOAD! (FATAL!) (bit 24)\n");
  if(_cube_state & PC_STATE_BEYOND_HARD)
    printf("HARD LIMIT PASSED! (bit 25)\n");
  if(_cube_state & PC_STATE_BEYOND_SOFT)
    printf("SOFT LIMIT PASSED! (bit 26)\n");
  if(_cube_state & PC_STATE_POW_SETUP_ERR)
    printf("CURRENT SETUP ERROR! (FATAL!) (bit 27)\n");
}

/*
  Requests a value from a single module.
*/
void pc_request_value_from_module(struct module _mod, unsigned char msg[8]){
  int ret;
  char err_msg[80];
  unsigned int bsize = 80;
  //printf("WriteWaiting....");
  //fflush(stdout);
  ret = canWriteWait(_mod.handle, (PC_CANID_CMDGET + _mod.canID), msg, 2, can_flags, 500);
  if(ret==0){
    //printf("done (ret=%d)\n",(int)ret);
    return;
  }else{
    //printf("done (ret=%d :",(int)ret);
    canGetErrorText(ret, err_msg, bsize);
    printf("CAN error in request_value_from_module [%d]: %s \n",(int)_mod.canID,err_msg);
  }
  return;
}

void pc_request_value_from_module_nowait(struct module _mod, unsigned char msg[8]){
  int ret;
  char err_msg[80];
  unsigned int bsize = 80;
  //printf("WriteWaiting....");
  //fflush(stdout);
  ret = canWrite(_mod.handle, (PC_CANID_CMDGET + _mod.canID), msg, 2, can_flags);
  if(ret==0){
    //printf("done (ret=%d)\n",(int)ret);
    return;
  }else{
    //printf("done (ret=%d :",(int)ret);
    canGetErrorText(ret, err_msg, bsize);
    printf("CAN error in request_value_from_module_NW [%d]: %s \n",(int)_mod.canID,err_msg);
  }
  return;
}


/*
  Listen for module response. Returns the number of the module that 
  responded. Waits until response arrives, or time out (100 ms).
  Returns negative value for error (e.g time out)
*/
int pc_listen_for_response(canHandle h, void *msg){
  int ret;
  long id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  char err_msg[80];
  unsigned int bsize = 80;
  //printf("ReadWaiting....");
  //fflush(stdout);
  ret = canReadWait(h, &id, msg, &dlc, &flag, &time, 100);
  //printf("done (ret=%d) (id=%d)\n",(int)ret,(int)id);
  if (ret < 0){
    canGetErrorText(ret, err_msg, bsize);
    //printf("CAN error in listen_for_response [%d]: %s \n",(int)h,err_msg);
    return ret;
  }
  if ((id - PC_CANID_CMDPUT) < 0){
    if ((id - PC_CANID_CMDGET) < 0){
      return (id - PC_CANID_CMDACK);
    }else{
      return (id - PC_CANID_CMDGET);
    }
  }else{
    return (id - PC_CANID_CMDPUT);
  }
}


int pc_listen_for_response_nowait(canHandle h, void *msg){
  int ret;
  long id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  char err_msg[80];
  unsigned int bsize = 80;
  //printf("ReadWaiting....");
  //fflush(stdout);
  ret = canRead(h, &id, msg, &dlc, &flag, &time);
  //printf("done (ret=%d)\n",(int)ret);
  if (ret < 0){
    canGetErrorText(ret, err_msg, bsize);
    printf("CAN error in listen_for_response_NW [%d]: %s \n",(int)h,err_msg);
    return ret;
  }
  if ((id - PC_CANID_CMDPUT) < 0){
    if ((id - PC_CANID_CMDGET) < 0){
      return (id - PC_CANID_CMDACK);
    }else{
      return (id - PC_CANID_CMDGET);
    }
  }else{
    return (id - PC_CANID_CMDPUT);
  }
}

char* pc_par2str(unsigned char parnum){
  switch(parnum){
  case PC_PARA_DEF_HOME_OFFSET:
    return "Default home offset";
    break;
  case PC_PARA_DEF_GEAR_RATIO:
    return "Default gear ratio";
    break;
  case PC_PARA_DEF_LIN_RATIO:
    return "Default linear ratio";
    break;
  case PC_PARA_DEF_MIN_POS:
    return "Default min pos";
    break;
  case PC_PARA_DEF_MAX_POS:
    return "Default max pos";
    break;
  case PC_PARA_DEF_MAX_DELTA_POS:
    return "Default max delta pos (tow error)";
    break;
  case PC_PARA_DEF_MAX_DELTA_VEL:
    return "Default max delta vel (follow error)";
    break;
  case PC_PARA_DEF_TORQUE_RATIO:
    return "Default torque/current ratio";
    break;
  case PC_PARA_DEF_CUR_RATIO:
    return "Default current ratio";
    break;
  case PC_PARA_DEF_MIN_VEL:
    return "Default min vel";
    break;
  case PC_PARA_DEF_MAX_VEL:
    return "Default max vel";
    break;
  case PC_PARA_DEF_MIN_ACC:
    return "Default min acc";
    break;
  case PC_PARA_DEF_MAX_ACC:
    return "Default max acc";
    break;
  case PC_PARA_DEF_MIN_CUR:
    return "Default min current";
    break;
  case PC_PARA_DEF_MAX_CUR:
    return "Default max current";
    break;
  case PC_PARA_DEF_HOME_VEL:
    return "Default homing vel";
    break;
  case PC_PARA_DEF_HOME_ACC:
    return "Default homing acc";
    break;
  case PC_PARA_DEF_CUBE_SERIAL:
    return "Default serial number";
    break;
  case PC_PARA_DEF_CONFIG:
    return "Default config word";
    break;
  case PC_PARA_DEF_PULSES_PER_TURN:
    return "Default encoder ticks per revolution";
    break;
  case PC_PARA_DEF_CUBE_VERSION:
    return "Default cube version";
    break;
  case PC_PARA_DEF_SERVICE_INTERVAL:
    return "Default service interval";
    break;
  case PC_PARA_DEF_BRAKE_TIME_OUT:
    return "Default brake release delay";
    break;
  case PC_PARA_DEF_ADDRESS:
    return "Default bus adress";
    break;
  case PC_PARA_DEF_PRIM_BAUD:
    return "Default primary baud setting";
    break;
  case PC_PARA_DEF_SCND_BAUD:
    return "Default secondary baud setting";
    break;
  case PC_PARA_POS_COUNT:
    return "Absolute counter value";
    break;
  case PC_PARA_REF_POS_COUNT:
    return "Absolute counter value at home";
    break;
  case PC_PARA_DIO_SETUP:
    return "Digital IO word";
    break;
  case PC_PARA_CUBE_STATE:
    return "Module state word";
    break;
  case PC_PARA_TARGET_POS_INC:
    return "Target pos in encoder ticks";
    break;
  case PC_PARA_TARGET_VEL_INC:
    return "Target vel in ticks/s";
    break;
  case PC_PARA_TARGET_ACC_INC:
    return "Target acc in ticks/(s*s)";
    break;
  case PC_PARA_STEP_INC:
    return "Step mode target pos in ticks";
    break;
  case PC_PARA_HOME_OFFSET_INC:
    return "Home offset in ticks";
    break;
  case PC_PARA_RAW_CUR:
    return "Commanded current in digits";
    break;
  case PC_PARA_HOME_TO_ZERO_INC:
    return "Number of ticks from home and encoder index";
    break;
  case PC_PARA_CONFIG:
    return "Config word, actual value";
    break;
  case PC_PARA_MOVE_MODE:
    return "Motion mode";
    break;
  case PC_PARA_INC_RATIO:
    return "Ratio ticks/units";
    break;
  case PC_PARA_ACT_POS:
    return "Actual pos";
    break;
  case PC_PARA_ACT_POS_:
    return "Previous pos";
    break;
  case PC_PARA_IPOL_POS:
    return "Actual interpolated position";
    break;
  case PC_PARA_DELTA_POS:
    return "Actual following error";
    break;
  case PC_PARA_MAX_DELTA_POS:
    return "Maximum following error (limit)";
    break;
  case PC_PARA_ACT_VEL:
    return "Actual vel";
    break;
  case PC_PARA_IPOL_VEL:
    return "Interpolated actual vel";
    break;
  case PC_PARA_MIN_POS:
    return "Min pos (limit)";
    break;
  case PC_PARA_MAX_POS:
    return "Max pos (limit)";
    break;
  case PC_PARA_MAX_VEL:
    return "Max vel (limit)";
    break;
  case PC_PARA_MAX_ACC:
    return "Max acc (limit)";
    break;
  case PC_PARA_MAX_CUR:
    return "Max current (limit)";
    break;
  case PC_PARA_CUR:
    return "Actual current";
    break;
  case PC_PARA_TARGET_POS:
    return "Target pos (units)";
    break;
  case PC_PARA_TARGET_VEL:
    return "Target vel (units/s)";
    break;
  case PC_PARA_TARGET_ACC:
    return "Target acc (units/(s*s))";
    break;
  case PC_PARA_DEF_C0:
    return "Servo loop gain C0 default";
    break;
  case PC_PARA_DEF_DAMP:
    return "Servo loop damping default";
    break;
  case PC_PARA_DEF_A0:
    return "Servo loop A0 default";
    break;
  case PC_PARA_ACT_C0:
    return "Servo loop gain C0";
    break;
  case PC_PARA_ACT_DAMP:
    return "Servo loop damping";
    break;
  case PC_PARA_ACT_A0:
    return "Servo loop A0";
    break;
  case PC_PARA_DEF_BURN_COUNT:
    return "Burn count";
    break;
  case PC_PARA_SETUP:
    return "Default setup word";
    break;
  case PC_PARA_HOME_OFFSET:
    return "Actual home offset";
    break;
  default:
    return "UNDEFINED";
  }
}

void pc_display_message(unsigned char *msg){
  int i;
  unsigned char u8data;
  char i8data;
  unsigned short int u16data;
  short int i16data;
  long int i32data;  
  unsigned long int u32data;  
  float fdata;
  printf("Type was %s, ",pc_par2str(msg[1]));
  switch (pc_get_data_type(msg[0],msg[1])){
  case PC_DATA_TYPE_FLOAT:
    get_data_float(msg,&fdata);
    printf("Data was %f",fdata);
    break;
  case PC_DATA_TYPE_INT8:
    i8data=msg[2];
    printf("Data was %d",i8data);
    break;
  case PC_DATA_TYPE_UINT8:
    u8data=msg[2];
    printf("Data was %d",u8data);
    break;
  case PC_DATA_TYPE_UINT16:
    get_data_uint16(msg,&u16data);
    printf("Data was %d",u16data);
    break;
  case PC_DATA_TYPE_INT16:
    get_data_int16(msg,&i16data);
    printf("Data was %d",i16data);
    break;
  case PC_DATA_TYPE_UINT32:
    get_data_uint32(msg,&u32data);
    printf("Data was %ld (bits: ",u32data);
    for(i=0;i<32;i++){
      if(u32data&(0x1<<(31-i))){
	printf("1");
      }else
	printf("0");
      if(((i+1)%4)==0){
	printf(" ");
      }
    }
    printf(")");
    break;
  case PC_DATA_TYPE_INT32:
    get_data_int(msg,&i32data);
    printf("Data was %ld",i32data);
    break;
  case PC_DATA_TYPE_FLOAT_UINT16:
    get_data_float_uint16(msg,&fdata,&u16data);
    printf("Data was %f %d",fdata,u16data);
    break;
  case PC_DATA_TYPE_INT32_UINT16:
    get_data_int_uint16(msg,&i32data,&u16data);
    printf("Data was %ld %d",i32data,u16data);
    break;
  }
}


void *watchdog_thread(void *arg){

  struct timespec remain;
  struct timespec nsleep_time;
  struct module _mod;
  struct module *_modp;

  _modp = (struct module *)arg;
  _mod = *_modp;
  
  printf("Watchdog starting!\n");

  nsleep_time.tv_sec = 0;
  nsleep_time.tv_nsec = PC_WATCHDOG_TIMER_PERIOD_NS;

  while(!kill_watchdog){
    pc_watch_refresh_all_on_bus(_mod.handle);
    nanosleep(&nsleep_time,&remain);
  }
  printf("Watchdog killed!\n");
  return NULL;
}

void pc_enable_watchdog(struct module _mod){
  
  unsigned char msg[8];
  unsigned int dlc;
  unsigned long int config_word;

  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CONFIG;
  pc_request_value_from_module(_mod, msg);
  pc_listen_for_response(_mod.handle,&msg);
  get_data_uint32(msg, &config_word);
  if(!(config_word & PC_CONFIGID_MOD_WATCHDOG_ENABLE)){
    msg[0] = PC_ALL_COMMAND_WATCHDOG_REFRESH;  
    dlc = 1;
    pc_send_command_to_module(_mod, msg, dlc);
  }
}

pthread_t pc_start_watchdog_thread(struct module *_mod){
  pthread_t pth;

  kill_watchdog = 0;
  pthread_create(&pth, NULL, watchdog_thread,(void *)(_mod));
  return pth;
}


void pc_kill_watchdog(pthread_t pth){
  struct timespec remain;
  struct timespec nsleep_time; 

  nsleep_time.tv_sec = 0;
  nsleep_time.tv_nsec = PC_WATCHDOG_TIMER_PERIOD_NS*3;
  
  kill_watchdog = 1;
  nanosleep(&nsleep_time,&remain);  
  pthread_detach(pth);
}

// Opens all 6 Can channels, set default parameters and goes on bus
void pc_open_init_onBus(canHandle h[6]){
  int _channel;
  for(_channel=0;_channel<6;_channel++){
    h[_channel] = canOpenChannel(_channel, canWANT_EXCLUSIVE | canWANT_EXTENDED);
    if (h[_channel] < 0) {
      char msg[64];
      canGetErrorText(h[_channel], msg, sizeof(msg));
      printf("canOpenChannel %d failed with error %s\n", _channel, msg);
      perror("pc_open_init_oBbus");
      exit(EXIT_FAILURE);
    }
    
    canSetBusParams(h[_channel], BAUD_250K, 4, 3, 1, 1, 0);
    canBusOn(h[_channel]);
  }
}


// Sets all powercubes and CAN channels to 250kBaud
void pc_setAll250k(canHandle h[6]){
  int _j;
  for(_j=0;_j<6;_j++){
    printf("  Resetting PC baudrate....");
    fflush(stdout);

    canBusOff(h[_j]);
    canSetBusParams(h[_j], BAUD_1M, 4, 3, 1, 1, 0);
    pc_set_baud_all_on_bus(h[_j], PC_BAUDRATE_250K);
    canBusOn(h[_j]);
    /*
    canBusOff(h[_j]);
    canSetBusParams(h[_j], BAUD_500K, 4, 3, 1, 1, 0);
    pc_set_baud_all_on_bus(h[_j], PC_BAUDRATE_250K);
    canBusOn(h[_j]);
    */
    canBusOff(h[_j]);
    canSetBusParams(h[_j], BAUD_250K, 4, 3, 1, 1, 0);
    pc_set_baud_all_on_bus(h[_j], PC_BAUDRATE_250K);
    canBusOn(h[_j]);
    

    printf("Done\n");
  }
}

// Initialize modules.
void pc_initialize_powercubes(struct module cube[6], canHandle h[6]){
  int _j;
  int _channel;
  unsigned char _msg[8];
  for(_j=0;_j<6;_j++){
    cube[_j].canID=0;
  }
  for(_channel=0;_channel<6;_channel++){
    for(_j=0;_j<4;_j++){
      struct module test_cube;
      int bussAdress;
      unsigned long int serial;
      test_cube.handle = h[_channel];
      test_cube.canID = _j+1;
      _msg[0] = PC_COMMAND_GET_EXTENDED;
      _msg[1] = PC_PARA_DEF_ADDRESS;
      pc_request_value_from_module(test_cube, _msg);
      bussAdress = pc_listen_for_response(h[_channel],&_msg);
      //printf("can channel:%d, response:%d, j:%d\n",_channel,bussAdress, _j);
      if(bussAdress<4 && bussAdress>1){
	//printf("Cube %d answered on channel %d",bussAdress,_channel);
	cube[_j].handle=h[_channel];
	cube[_j].canID=_j+1;
	break;
      }
      if(bussAdress == 4){
	cube[3].handle = h[_channel];
	cube[3].canID = 4;
	cube[4].handle = h[_channel];
	cube[4].canID = 5;
	cube[5].handle = h[_channel];
	cube[5].canID = 6;
	pc_listen_for_response(h[_channel],&_msg); // clears CAN buffer
	pc_listen_for_response(h[_channel],&_msg);
	break;
      }
      if(bussAdress==1){
	//printf("Cube %d answered on channel %d. test_cube handle: %d \n",bussAdress,_channel,test_cube.canID);
        _msg[0] = PC_COMMAND_GET_EXTENDED;
        _msg[1] = PC_PARA_DEF_CUBE_SERIAL;
	pc_request_value_from_module(test_cube, _msg);
        //pc_send_command_to_module(test_cube, _msg, 2);
	pc_listen_for_response(test_cube.handle, &_msg);
        //pc_listen_for_response(h[_channel],&_msg);
        //{int __i;
        // for(__i=0;__i<8;__i++){
	//printf("byte %d: %u\n",__i,(unsigned int)_msg[__i]);
	//}
        //}
        get_data_uint32(_msg,&serial);
	//printf("Cube %d has serial %d (popeye:%d   dumbo:%d)\n",bussAdress,serial,DEF_POPEYE_SERIAL_NUMBER,DEF_DUMBO_SERIAL_NUMBER);
        if (serial == DEF_POPEYE_SERIAL_NUMBER){
          printf("Found Popeye! \n");
	   cube[_j].handle=h[_channel];
	   cube[_j].canID=_j+1;
	   //break;
        }
	pc_listen_for_response(h[_channel],&_msg); // clears CAN buffer
	pc_listen_for_response(h[_channel],&_msg);
        break;
      }
    }
    pc_listen_for_response(h[_channel],&_msg); // clears CAN buffer
  }
  
  printf("Initialization results of Popeye:\n");
  for(_j=0;_j<6;_j++){
    printf("  Cube %d has CAN channel %ld\n",_j,cube[_j].canID);
  }
}

// Home and reset all modules
// After homing, module 3 is set to -20 degrees to be within legal initial limits
void pc_home_and_reset_modules(struct module cube[6]){
  int _j,_jj;  
  unsigned int _dlc;
  for(_jj=0;_jj<2;_jj++){
    struct timespec nsleep_time;
    struct timespec remain;
    int homing_in_progress = 1;
    unsigned char _msg[8];

    nsleep_time.tv_sec = 0;
    nsleep_time.tv_nsec = 200000000;
    printf("Homing Popeye\n");
    if(_jj==1){
      for(_j=0;_j<MAX_POPEYE_CUBES;_j++){
        printf("Homing module %d.\n",_j+1);
        nanosleep(&nsleep_time,&remain);
        pc_home_module(cube[_j]);
        pc_listen_for_response(cube[_j].handle,&_msg);
      }

      nsleep_time.tv_sec = 0;
      nsleep_time.tv_nsec = 500000000;

      homing_in_progress = 1;
      while(homing_in_progress){
        homing_in_progress = 0;
        for(_j=0;_j<MAX_POPEYE_CUBES;_j++){
          nanosleep(&nsleep_time,&remain);
          if(!(pc_get_status(cube[_j]) & PC_STATE_HOME_OK)){
	   // printf("\nCube %d homed\n",_j);
            homing_in_progress++;
          }
        }
      }
      printf("Homed!\n");
      fflush(stdout);
    }

    for(_j=0;_j<MAX_POPEYE_CUBES;_j++){
      float fvalue;
      unsigned char nulmsg[8];
      printf("Resetting module %d.\n",_j+1);
      nanosleep(&nsleep_time,&remain);
      pc_reset_module(cube[_j]);
      _msg[0]=PC_COMMAND_SET_EXTENDED;
      _msg[1]=PC_PARA_TARGET_ACC;
      if(_j<3){
	fvalue = 0.8;//INNER_CUBE_VEL;
      }else{
	fvalue = 1.0;//INNER_CUBE_VEL;
      }
      _dlc = set_data_float(_msg,&fvalue);
      pc_send_command_to_module(cube[_j], _msg, _dlc);
      pc_listen_for_response(cube[_j].handle,&nulmsg);
      _msg[0]=PC_COMMAND_SET_EXTENDED;
      _msg[1]=PC_PARA_TARGET_VEL;
      if(_j<3){
	fvalue = 0.2;//INNER_CUBE_VEL;
      }else{
	fvalue = 0.5;//INNER_CUBE_VEL;
      }
      _dlc = set_data_float(_msg,&fvalue);
      pc_send_command_to_module(cube[_j], _msg, _dlc);
      pc_listen_for_response(cube[_j].handle,&nulmsg);
      pc_listen_for_response(cube[_j].handle,&nulmsg);
      if(_j==2 && _jj==1){
	float cur_pos = 0;
        fvalue = -20.0*PI/180.0;
        _msg[0] = PC_COMMAND_SET_MOTION;
        _msg[1] = PC_MOTION_FRAMP_ACK;
        _dlc = set_data_float(_msg, &fvalue);
        pc_send_command_to_module(cube[_j], _msg, _dlc);
        pc_listen_for_response(cube[_j].handle,&nulmsg);
        printf("(target=%6.2f) Currently in pos %6.2f",fvalue*180/PI,cur_pos*180/PI);

        while(cur_pos>(-19*PI/180)){
          nsleep_time.tv_sec = 0;
          nsleep_time.tv_nsec = 20000000;

          get_data_float(nulmsg, &cur_pos);
          printf("\b\b\b\b\b\b");
          printf("%6.2f",cur_pos*180/PI);
          fflush(stdout);
          nanosleep(&nsleep_time,&remain);
          _msg[0] = PC_COMMAND_GET_EXTENDED;
	  _msg[1] = PC_PARA_ACT_POS;
          pc_request_value_from_module(cube[_j],_msg);
          pc_listen_for_response(cube[_j].handle,&nulmsg);
        }
        printf("\n");
      }
    }
  }
}



// set angle limits
void pc_set_angle_limits(struct module cube[6],
			 float cubelimit_lower[6],
			 float cubelimit_upper[6]){

  int _j;
  unsigned int _dlc;
  unsigned char _msg[8];
  unsigned char _ret_msg[8];
  float lower_limit;
  float upper_limit;
  printf("Setting angle limits for Popeye\n");
  for(_j=0;_j<MAX_POPEYE_CUBES;_j++){
    printf("Setting limits:\n");
    lower_limit = cubelimit_lower[_j];
    upper_limit = cubelimit_upper[_j];

    _msg[0]=PC_COMMAND_SET_EXTENDED;
    _msg[1]=PC_PARA_MIN_POS;
    _dlc = set_data_float(_msg,&lower_limit);
    pc_send_command_to_module(cube[_j], _msg, _dlc);

    _ret_msg[2]=0;
    pc_listen_for_response(cube[_j].handle, &_ret_msg);
    if(!(_ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
           _ret_msg[1]==PC_PARA_MIN_POS &&
	 _ret_msg[2]==0X64)){
      printf("Failed! (none/incorrect response from cube)  %X %X %X",
	     _ret_msg[0],_ret_msg[1],_ret_msg[2]);
    }else{
      printf("OK.(%6.2f)..",lower_limit);
    }

    _msg[1]=PC_PARA_MAX_POS;
    _dlc = set_data_float(_msg,&upper_limit);
    pc_send_command_to_module(cube[_j], _msg, _dlc);


    _ret_msg[2]=0;
    pc_listen_for_response(cube[_j].handle, &_ret_msg);
    if(!(_ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
           _ret_msg[1]==PC_PARA_MAX_POS &&
	 _ret_msg[2]==0X64)){
      printf("Failed! (none/incorrect response from cube)  %X %X %X",
	     _ret_msg[0],_ret_msg[1],_ret_msg[2]);
    }else{
      printf("..OK (%6.2f)\n",upper_limit);
    }
    fflush(stdout);
  }
}

