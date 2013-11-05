/*
 * powerarm.c
 *
 *  Created on: Dec 2009
 *  Authors:   Christian Smith
 *            ccs <at> kth.se 
 */

/* Copyright (c) 2009, Christian Smith, KTH
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


// Coded by Christian Smith, Dec 2009
// Instructions can be found in powerarm.h

#include <dumbo_powercube_chain/powerarm.h>
#include <kvaser_canlib/canlib.h>
#include <dumbo_powercube_chain/powercube_commands.h>
#include <dumbo_powercube_chain/powercube_defines.h>
/* #include "ft_sensor_function.h" */
/* #include "powercube_dumbo_kinematics.h" */
#include <dumbo_powercube_chain/dumbo_arm_params.h>
#include <dumbo_powercube_chain/collision.h>
//#include "powercube_position_com.h"
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <math.h>

#define INIT_VEL (5*(PI/180))
/* #define INIT_ACC (50*(PI/180)) */
#define INIT_ACC 0.8

/* #define MAX_VEL (39.9*(PI/180)) */
#define MAX_VEL (5*(PI/180))

canHandle h[2];

// This contains data for the modules
static struct module dumbo_cube_L[8];
static struct module dumbo_cube_R[8];

static struct joint_t last_angle_L;
static struct joint_t last_angle_R;

int gripper_exists_L = 0;
int gripper_exists_R = 0;

// watchdog threads
static pthread_t WD_thread_L;
static pthread_t WD_thread_R;

// FT stuff...
int sensorChannel = -1;
float calibration_matrix[6][6];
// These are measured from th left arm:
double bias_val[] = {11.6, 3.0, -1.4, 0.265, -0.34, 0.1};


static float lower_limit_L[] = {LOWER_CUBE_LIMIT_1,
				LOWER_CUBE_LIMIT_2,
				LOWER_CUBE_LIMIT_3,
				LOWER_CUBE_LIMIT_4,
				LOWER_CUBE_LIMIT_5,
				LOWER_CUBE_LIMIT_6,
				LOWER_CUBE_LIMIT_7,
				LOWER_CUBE_LIMIT_G}; // gripper

static float upper_limit_L[] = {UPPER_CUBE_LIMIT_1,
				UPPER_CUBE_LIMIT_2,
				UPPER_CUBE_LIMIT_3,
				UPPER_CUBE_LIMIT_4,
				UPPER_CUBE_LIMIT_5,
				UPPER_CUBE_LIMIT_6,
				UPPER_CUBE_LIMIT_7,
				UPPER_CUBE_LIMIT_G}; // Gripper

static float lower_limit_R[] = {LOWER_CUBE_LIMIT_1,
				LOWER_CUBE_LIMIT_2,
				LOWER_CUBE_LIMIT_3,
				LOWER_CUBE_LIMIT_4,
				LOWER_CUBE_LIMIT_5,
				LOWER_CUBE_LIMIT_6,
				LOWER_CUBE_LIMIT_7,
				LOWER_CUBE_LIMIT_G}; // gripper

static float upper_limit_R[] = {UPPER_CUBE_LIMIT_1,
				UPPER_CUBE_LIMIT_2,
				UPPER_CUBE_LIMIT_3,
				UPPER_CUBE_LIMIT_4,
				UPPER_CUBE_LIMIT_5,
				UPPER_CUBE_LIMIT_6,
				UPPER_CUBE_LIMIT_7,
				UPPER_CUBE_LIMIT_G}; // Gripper



static float vel_limit_L[] ={0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0,
			     0.0}; 

static float vel_limit_R[] ={0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0,
			     0.0}; 

static float acc_limit_L[] ={0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0,
			     0.0}; 

static float acc_limit_R[] ={0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0, 
			     0.0,
			     0.0};

static char string_LR[2][2] = {{"L"},{"R"}}; 


// Connect to Arms
int pa_connect(arm_select LR){

  int i;
  int channel;
  int ret = 0;
  unsigned long int cube_state=0;
  unsigned long int serial;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  int found = 0;

  struct module *dumbo_cube;
  struct joint_t *last_angle;
  int *gripper_exists;

  //int bitrate250k = BAUD_250K;

  if((LR!=LEFT_ARM)&&(LR!=RIGHT_ARM)){
    printf("Connect error: no arm selected...\n");
    return -1;
  }

  switch (LR){

  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    last_angle = &last_angle_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    last_angle = &last_angle_R;
    gripper_exists = &gripper_exists_R;
    break;

  }
    

  for(i=0;i<8;i++){
    dumbo_cube[i].canID=0;
  }

  for(channel = 0;channel<2;channel++){
    h[channel] = canOpenChannel(channel, canWANT_EXCLUSIVE |canWANT_EXTENDED);
    canSetBusParams(h[channel], BAUD_500K,4,3,1,1,0);
    canBusOn(h[channel]);
  }

  printf("looking for dumbo\n");
  // Find arm
  for(channel=0;channel<2;channel++){
    struct module test_cube;
    int bussAdress;

    ret = -2; // if this step fails, return -2
    test_cube.handle = h[channel];
    test_cube.canID = 1;
    msg[0] = PC_COMMAND_GET_EXTENDED;
    msg[1] = PC_PARA_DEF_ADDRESS;
    pc_send_command_to_module(test_cube, msg, 2);
    bussAdress = pc_listen_for_response(h[channel],&msg);
    printf("can channel:%d, response:%d\n",channel,bussAdress);
    if(bussAdress==1){
      msg[0] = PC_COMMAND_GET_EXTENDED;
      msg[1] = PC_PARA_DEF_CUBE_SERIAL;
      pc_request_value_from_module(test_cube, msg);
      //pc_send_command_to_module(test_cube, _msg, 2);
      pc_listen_for_response(test_cube.handle, &ret_msg);
      //pc_listen_for_response(h[_channel],&_msg);
        
      //pc_send_command_to_module(test_cube, msg, 2);
      //pc_listen_for_response(h[channel],&msg);
      get_data_uint32(ret_msg,&serial);
      printf("BussAdress %d has serial %u (left arm:%u   right arm:%u)\n",
	     bussAdress,(unsigned int)serial,
	     DEF_SCHUNK_LEFTARM_SERIAL_NUMBER,
	     DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER);

      switch(LR){
      case LEFT_ARM:
        if((int)serial == DEF_SCHUNK_LEFTARM_SERIAL_NUMBER){
          printf("Connected to Dumbo left arm! \n");
          for(i=0;i<8;i++){
	    dumbo_cube[i].handle = h[channel];
            dumbo_cube[i].canID = i+1;
	  }
	  found += 1;
        }
	break;

      case RIGHT_ARM:
        if((int)serial == DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER){
          printf("Connected to Dumbo right arm! \n");
          for(i=0;i<8;i++){
	    dumbo_cube[i].handle = h[channel];
            dumbo_cube[i].canID = i+1;
	  }
	  found += 1;
        }
	break;
      }

      // ret = 0;
      //break;
    }
    if(found==1){
      ret = 0;
    }

    pc_listen_for_response(h[channel],&msg); // clears CAN buffer
    //	pc_listen_for_response(h[channel],&msg);
  }


  // Check status of cubes.
  // make sure there is no error.
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CUBE_STATE;


  for(i=0;i<7;i++){
    last_angle->j[i] = 0.0; // initialize last_angle value
    pc_request_value_from_module(dumbo_cube[i],msg);
    found = pc_listen_for_response(dumbo_cube[i].handle,&ret_msg);
    printf("module L%d has channel %d\n",i,found);
    get_data_uint32(ret_msg,&cube_state);
    if(cube_state & PC_STATE_ERROR){
      ret = -2;
      printf("Error on module L%d\n",i);
    }
  }

  i = 7; // Gripper
  
  pc_request_value_from_module(dumbo_cube[i],msg);
  found = pc_listen_for_response(dumbo_cube[i].handle,&ret_msg);
  printf("response from left gripper: %d\n",found);
  get_data_uint32(ret_msg,&cube_state);
  last_angle->gripper = 0.0; // initialize last_angle value
  if((found==8) && (cube_state & PC_STATE_ERROR)){ // found = 8 means found...
    printf("Left gripper reports error!\n");
    ret = -2;
  }else if(found==8){
    *gripper_exists = 1;
  }
   

  printf("gripper(s) found:  %s:%d\n",
	 string_LR[LR], *gripper_exists);

  return ret;

}


// Connect to Arm & sensor
/* int pa_connect(void){ */

/*   int i,j; */
/*   int channel; */
/*   int ret = 0; */
/*   unsigned long int cube_state=0; */
/*   unsigned long int serial; */
/*   unsigned char msg[8]; */
/*   unsigned char ret_msg[8]; */
/*   int found = 0; */

/*   //int bitrate250k = BAUD_250K; */

/*   for(i=0;i<8;i++){ */
/*     dumbo_cube_L[i].canID=0; */
/*   } */

/*   for(channel = 0;channel<2;channel++){ */
/*     h[channel] = canOpenChannel(channel, canWANT_EXCLUSIVE |canWANT_EXTENDED);  */
/*     canSetBusParams(h[channel], BAUD_500K,4,3,1,1,0); */
/*     canBusOn(h[channel]); */
/*   } */

/*  printf("looking for dumbo\n"); */
/*   // Find arm */
/*   for(channel=0;channel<2;channel++){ */
/*     struct module test_cube; */
/*     int bussAdress; */

/*     ret = -2; // if this step fails, return -2 */
/*     test_cube.handle = h[channel]; */
/*     test_cube.canID = 1; */
/*     msg[0] = PC_COMMAND_GET_EXTENDED; */
/*     msg[1] = PC_PARA_DEF_ADDRESS; */
/*     pc_send_command_to_module(test_cube, msg, 2); */
/*     bussAdress = pc_listen_for_response(h[channel],&msg); */
/*     printf("can channel:%d, response:%d\n",channel,bussAdress); */
/*     if(bussAdress==1){ */
/*         msg[0] = PC_COMMAND_GET_EXTENDED; */
/*         msg[1] = PC_PARA_DEF_CUBE_SERIAL; */
/* 	pc_request_value_from_module(test_cube, msg); */
/*         //pc_send_command_to_module(test_cube, _msg, 2); */
/* 	pc_listen_for_response(test_cube.handle, &ret_msg); */
/*         //pc_listen_for_response(h[_channel],&_msg); */
        
/*         //pc_send_command_to_module(test_cube, msg, 2); */
/*         //pc_listen_for_response(h[channel],&msg); */
/*         get_data_uint32(ret_msg,&serial); */
/* 	printf("BussAdress %d has serial %u (left arm:%u   right arm:%u)\n",bussAdress,(unsigned int)serial, DEF_SCHUNK_LEFTARM_SERIAL_NUMBER, DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER); */
/*         if((int)serial == DEF_SCHUNK_LEFTARM_SERIAL_NUMBER){ */
/*           printf("Connected to Dumbo left arm! \n"); */
/*           for(i=0;i<8;i++){ */
/* 	    dumbo_cube_L[i].handle = h[channel]; */
/*             dumbo_cube_L[i].canID = i+1; */
/* 	  }  */
/* 	  found += 1; */
/* 	  sensorChannel = 1-channel; */
/*         } */
/*         if((int)serial == DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER){ */
/*           printf("Connected to Dumbo right arm! \n"); */
/* 	  found += 10; */
/*         } */


/*      // ret = 0; */
/*      //break; */
/*     } */
/*     if(found==1){ */
/*       ret = 0; */
/*     } */

/*     pc_listen_for_response(h[channel],&msg); // clears CAN buffer */
/* //	pc_listen_for_response(h[channel],&msg); */
/*   } */


/*   // Check status of cubes. */
/*   // make sure there is no error. */
/*   msg[0] = PC_COMMAND_GET_EXTENDED; */
/*   msg[1] = PC_PARA_CUBE_STATE; */
/*   for(i=0;i<7;i++){ */
/*     last_angle_L.j[i] = 0.0; // initialize last_angle value */
/*     pc_request_value_from_module(dumbo_cube_L[i],msg); */
/*     found = pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg); */
/*     printf("module L%d has channel %d\n",i,found); */
/*     get_data_uint32(ret_msg,&cube_state); */
/*     if(cube_state & PC_STATE_ERROR){ */
/*       ret = -2; */
/*       printf("Error on module L%d\n",i); */
/*     } */
/*   } */

/*   i = 7; // Gripper */
/*   pc_request_value_from_module(dumbo_cube_L[i],msg); */
/*   found = pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg); */
/*   printf("response from left gripper: %d\n",found); */
/*   get_data_uint32(ret_msg,&cube_state); */
/*   last_angle_L.gripper = 0.0; // initialize last_angle value */
/*   if((found==8) && (cube_state & PC_STATE_ERROR)){ // found = 8 means found... */
/*       printf("Left gripper reports error!\n"); */
/*       ret = -2; */
/*   }else if(found==8){ */
/*     gripper_exists_L = 1; */
/*   } */
/*   printf("gripper(s) found:  L:%d\n", */
/* 	 gripper_exists_L); */

  
/*   printf("Initialising sensor:\n"); */
/*   canBusOff(h[sensorChannel]); */
/*   canClose(h[sensorChannel]); */

/*   h[sensorChannel] = canOpenChannel(sensorChannel, canWANT_EXCLUSIVE); */
/*   canSetBusParams(h[sensorChannel], BAUD_250K, 0, 0, 0, 0, 0);  */
/*   canSetBusOutputControl(h[sensorChannel], canDRIVER_NORMAL);  */
/*   canBusOn(h[sensorChannel]); */
/*   get_Transducer_CalMatrix(h[sensorChannel],&calibration_matrix[0][0]); */
/*   printf("Calibration matrix:\n"); */
/*   for(i = 0; i<6; i++){ */
/*     for(j = 0; j<6; j++){ */
/*       printf("%10.8f ",calibration_matrix[i][j]); */
/*     } */
/*     printf("\n"); */
/*   } */

/*   return ret; */
/* } */



// Disconnect CAN
int pa_disconnect(void){
  int ret = 0;
  int channel;  

  for(channel=0;channel<2;channel++){
    ret += canClose(h[channel]);
  }

  return ret;
}


//starts the watchdog
pthread_t pa_start_watchdog(arm_select LR){

  int i;
  struct module *dumbo_cube;
  int *gripper_exists;  
  pthread_t *WD_thread;
  
  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    WD_thread = &WD_thread_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    WD_thread = &WD_thread_R;
    break;
  }
  
  for(i=0;i<7+(*gripper_exists);i++){
    pc_enable_watchdog(dumbo_cube[i]);
  }

  *WD_thread = pc_start_watchdog_thread(dumbo_cube);
  
  return *WD_thread;  

}

int pa_home_L(void){
  int ret = 0;
  int i;
  unsigned long int cube_state;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  struct timespec nsleep_time;
  struct timespec remain;
  int homed = 0;
  int counter = 0;

  //used for storing which modules require homing
  int list[8];
  int n,j;


  // determines which modules require homing
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CUBE_STATE;
  
  n = 0;

  for(i=0;i<7+gripper_exists_L;i++){
    pc_request_value_from_module(dumbo_cube_L[i],msg);
    pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg);
    get_data_uint32(ret_msg,&cube_state);
    if(!(cube_state & PC_STATE_HOME_OK)){
      list[n] = i;
      n++;	
    }
  }

  printf("attempting to home left arm\n");

  // send home command to all cubes which are not already homed:
  for(i=0;i<n;i++){
    j = list[i];
    pc_home_module(dumbo_cube_L[j]);
    if((ret=pc_listen_for_response(dumbo_cube_L[j].handle,&msg))!=(j+1)){
      
      return ret;

    }else{
      ret = 0;
    }
      
  }

  // wait until all cubes are homed,
  // or timed out (100 s)
  nsleep_time.tv_sec = 0;
  nsleep_time.tv_nsec = 500000000;
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CUBE_STATE;
  while(!homed && counter++ < 200){
    nanosleep(&nsleep_time,&remain);
    homed = 1;
    for(i=0;i<(7+gripper_exists_L);i++){
      pc_request_value_from_module(dumbo_cube_L[i],msg);
      pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg);
      get_data_uint32(ret_msg,&cube_state);
      if(cube_state & PC_STATE_ERROR){
	ret = -1;
      }
      if(!(cube_state & PC_STATE_HOME_OK))
	homed = 0;
    }
  }

  if((!homed) || (counter>=200))
    ret = -1;

  return ret;
}

int pa_home_R(void){
  int ret = 0;
  int i;
  unsigned long int cube_state;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  struct timespec nsleep_time;
  struct timespec remain;
  int homed = 0;
  int counter = 0;

  //used for storing which modules require homing
  int list[8];
  int n,j;


  // determines which modules require homing
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CUBE_STATE;
  
  n = 0;

  for(i=0;i<7+gripper_exists_R;i++){
    pc_request_value_from_module(dumbo_cube_R[i],msg);
    pc_listen_for_response(dumbo_cube_R[i].handle,&ret_msg);
    get_data_uint32(ret_msg,&cube_state);
    if(!(cube_state & PC_STATE_HOME_OK)){
      list[n] = i;
      n++;	
    }
  }

  printf("attempting to home right arm\n");

  // send home command to all cubes:
  for(i=0;i<n;i++){
    j=list[i];

    pc_home_module(dumbo_cube_R[j]);
    if((ret=pc_listen_for_response(dumbo_cube_R[j].handle,&msg))!=(j+1)){

      return ret;

    }else{
      ret = 0;
    }
      
  }

  // wait until all cubes are homed,
  // or timed out (100 s)
  nsleep_time.tv_sec = 0;
  nsleep_time.tv_nsec = 500000000;
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_CUBE_STATE;
  while(!homed && counter++ < 200){
    nanosleep(&nsleep_time,&remain);
    homed = 1;
    for(i=0;i<(7+gripper_exists_R);i++){
      pc_request_value_from_module(dumbo_cube_R[i],msg);
      pc_listen_for_response(dumbo_cube_R[i].handle,&ret_msg);
      get_data_uint32(ret_msg,&cube_state);
      if(cube_state & PC_STATE_ERROR){
	ret = -1;
      }
      if(!(cube_state & PC_STATE_HOME_OK))
	homed = 0;
    }
  }

  if((!homed) || (counter>=200))
    ret = -1;

  return ret;
}

int pa_home(arm_select LR){
  int ret = 0;
  
  switch(LR){
  case LEFT_ARM:
    ret = pa_home_L();
    break;

  case RIGHT_ARM:
    ret += pa_home_R();
    break;
  }

  if(ret<0) ret = -1;

  return ret;
}



int pa_reset_L(void){
  int ret = 0;
  int i;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  unsigned int dlc;
  float fvalue;


  // reset modules and set low velocity
  // and limits
  for(i=0;i<(7+gripper_exists_L);i++){
    //reset
    pc_reset_module(dumbo_cube_L[i]);
    if(pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("reset error: cube L%d\n",i);
    }
    //set vel
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_VEL;
    fvalue = INIT_VEL;//20*PI/180; // a fast value, 20 deg/s
    if(i==7){
      fvalue = 0.05; // gripper has different units, m/s (?)
    }
    vel_limit_L[i] = fvalue;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(dumbo_cube_L[i], msg, dlc);
    if(pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("set max vel error: cube L%d\n",i);
    }
    //set acc
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_ACC;
    if(i==7){
      fvalue = 0.2;
    }else{
      fvalue = INIT_ACC;
    }
    acc_limit_L[i] = fvalue;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(dumbo_cube_L[i], msg, dlc);
    if(pc_listen_for_response(dumbo_cube_L[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("set max acc error: cube L%d\n",i);
    }
    // set limits
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_MIN_POS;
    dlc = set_data_float(msg,&lower_limit_L[i]);
    pc_send_command_to_module(dumbo_cube_L[i], msg, dlc);
    ret_msg[2]=0;
    pc_listen_for_response(dumbo_cube_L[i].handle, &ret_msg);
    if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
	 ret_msg[1]==PC_PARA_MIN_POS &&
	 ret_msg[2]==0X64)){
      ret += -1;
      printf("set min lim error: cube L%d\n",i);
    }	
    msg[1]=PC_PARA_MAX_POS;
    dlc = set_data_float(msg,&upper_limit_L[i]);
    pc_send_command_to_module(dumbo_cube_L[i], msg, dlc);
    
    ret_msg[2]=0;
    pc_listen_for_response(dumbo_cube_L[i].handle, &ret_msg);
    if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
	 ret_msg[1]==PC_PARA_MAX_POS &&
	 ret_msg[2]==0X64)){
      ret += -1;
      printf("set max lim error: cube L%d\n",i);
    }

  }
  if(ret !=0) ret = -1;

  return ret;
}

int pa_reset_R(void){
  int ret = 0;
  int i;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  unsigned int dlc;
  float fvalue;
  
  
  // reset modules and set low velocity
  // and limits
  for(i=0;i<(7+gripper_exists_R);i++){
    //reset
    pc_reset_module(dumbo_cube_R[i]);
    if(pc_listen_for_response(dumbo_cube_R[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("reset error: cube R%d\n",i);
    }
      
    //set acc
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_ACC;
    if(i==7){
      fvalue = 0.03;
    }else{
      fvalue = INIT_ACC;
    }
    acc_limit_R[i] = fvalue;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(dumbo_cube_R[i], msg, dlc);
    if(pc_listen_for_response(dumbo_cube_R[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("set acc lim error: cube R%d\n",i);
    }
      
    //set vel
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_VEL;
    fvalue = INIT_VEL;//20*PI/180; // a fast value, 20 deg/s
    if(i==7){
      fvalue = 0.05; // gripper has different units, m/s (?)
    }
    vel_limit_R[i] = fvalue;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(dumbo_cube_R[i], msg, dlc);
    if(pc_listen_for_response(dumbo_cube_R[i].handle,&ret_msg)!=(i+1)){
      ret = -1;
      printf("set vel lim error: cube R%d\n",i);
    }
    // set limits
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_MIN_POS;
    dlc = set_data_float(msg,&lower_limit_R[i]);
    pc_send_command_to_module(dumbo_cube_R[i], msg, dlc);
    ret_msg[2]=0;
    pc_listen_for_response(dumbo_cube_R[i].handle, &ret_msg);
    if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
	 ret_msg[1]==PC_PARA_MIN_POS &&
	 ret_msg[2]==0X64)){
      ret += -1;
      printf("set minpos error: cube R%d\n",i);
    }	
    msg[1]=PC_PARA_MAX_POS;
    dlc = set_data_float(msg,&upper_limit_R[i]);
    pc_send_command_to_module(dumbo_cube_R[i], msg, dlc);
      
    ret_msg[2]=0;
    pc_listen_for_response(dumbo_cube_R[i].handle, &ret_msg);
    if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
	 ret_msg[1]==PC_PARA_MAX_POS &&
	 ret_msg[2]==0X64)){
      printf("set maxpos error: cube R%d\n",i);
      ret += -1;
    }
  }
  if(ret !=0) ret = -1;
  
  return ret;
}


int pa_reset(arm_select LR){
  int ret = 0;

  switch(LR){
  case LEFT_ARM:
    ret = pa_reset_L();
    break;

  case RIGHT_ARM:
    ret += pa_reset_R();
    break;
  }

  return ret;
}


int pa_detect_error(arm_select LR){

  int ret = 0;
  int i; 
  struct module *dumbo_cube;
  int *gripper_exists;
  unsigned long int status;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    break;
  }

  for(i=0;i<7;i++){
    status = pc_get_status(dumbo_cube[i]);

    // if error bit is set... return -1
    if(status & PC_STATE_ERROR){
      printf("Error in module %d.\n", i);
      pc_print_module_state(status);
      ret = -1;
      return ret;
    }
  }

  // check the gripper, in case it is
  // connected to this arm
  if(*gripper_exists){
    i = 7;
    status = pc_get_status(dumbo_cube[i]);
    if(status & PC_STATE_ERROR){
      printf("Error in gripper.\n");
      pc_print_module_state(status);
      ret = -1;
      return ret;
    }
  }

  return ret;
}


int pa_get_state(int moduleID, unsigned long int *state, arm_select LR){

  int i; 
  int ret = 0;
  struct module *dumbo_cube;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    break;
  }

  *state = pc_get_status(dumbo_cube[moduleID]);

    // if error bit is set... return -1
    if(*state & PC_STATE_ERROR){
      printf("Error in module %d.\n", i);
      pc_print_module_state(*state);
      ret = -1;
      return ret;
    }

    return ret;

}

int pa_goto_angle(struct joint_t theta, arm_select LR){
  int ret = 0;
  int i;
  int res;
  unsigned int dlc;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  float fangle;
  struct joint_t theta_c;
  struct timespec sleep_time;
  
  float *lower_limit;
  float *upper_limit;
  struct module *dumbo_cube;
  struct joint_t *last_angle;
  int *gripper_exists;
  
  switch(LR){
  case LEFT_ARM:
    lower_limit = lower_limit_L;
    upper_limit = upper_limit_L;

    dumbo_cube = dumbo_cube_L;

    last_angle = &last_angle_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    lower_limit = lower_limit_R;
    upper_limit = upper_limit_R;

    dumbo_cube = dumbo_cube_R;

    last_angle = &last_angle_R;
    gripper_exists = &gripper_exists_R;
    break;
  }

  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 1000000; // 1 ms

  // check for possible collisions
  ret = pa_col_detect(theta, LR);
  if(ret<0){
    printf("Collision error: invalid input theta.\n");
    return EINVAL;
  }

  /* i=7; */
  /* if(*gripper_exists){ */
  /*   if((theta.gripper<lower_limit[i]) || */
  /*      (theta.gripper>upper_limit[i])){ */
  /*     printf("error for %s gripper, val%f\n", string_LR[LR], theta.gripper); */
  /*     return EINVAL; */
  /*   } */
  /* } */

  // correct joint angle directions
  // according to the modules' coordinate systems
  theta_c = pa_correct_theta(theta);
  
  // set modules to given angles 
  // ***HARD CODED TO MOVE ONLY 7TH JOINT
  for(i=6;i<7;i++){
    // in order to not overload the CAN bus, we sleep between sends:
    nanosleep(&sleep_time,NULL);
    msg[0] = PC_COMMAND_SET_MOTION;
    msg[1] = PC_MOTION_FRAMP_ACK;
    // left arm
    fangle = (float)theta_c.j[i];
    dlc = set_data_float(msg, &fangle);
    pc_send_command_to_module(dumbo_cube[i], msg, dlc);

    if((res=pc_listen_for_response(dumbo_cube[i].handle, &ret_msg))!=(i+1)){
      ret = -1;
      printf("[%d:%d]",i,res);
    }else{
      get_data_float(ret_msg,&fangle);
      last_angle->j[i] = (double)fangle;
    }
  }

  /* if(*gripper_exists){ */
  /*   i=7; */
  /*   msg[0] = PC_COMMAND_SET_MOTION; */
  /*   msg[1] = PC_MOTION_FRAMP_ACK; */
  /*   fangle = (float)theta_c.gripper; */
  /*   dlc = set_data_float(msg, &fangle); */
  /*   pc_send_command_to_module(dumbo_cube[i], msg, dlc); */
  /*   if(pc_listen_for_response(dumbo_cube[i].handle, &ret_msg)!=(i+1)){ */
  /*     ret = -1; */
  /*     printf("."); */
  /*   }else{ */
  /*     get_data_float(ret_msg,&fangle); */
  /*     last_angle->gripper = (double)fangle; */
  /*   } */
  /* }   */

  // correct joint angle directions
  // according to the modules' coordinate systems
  *last_angle = pa_correct_theta(*last_angle);

  // clear CAN stack:
  if(ret!=0){
    pc_listen_for_response(dumbo_cube[0].handle, &ret_msg);
  }


  if(ret !=0){
    ret = -1;
    printf("goto_ang ret: %d\n",ret);
  }
  return ret;
}

// compares two sets of angles:
double comp_angles(struct joint_t ang_A,
		   struct joint_t ang_B){
  int i;
  double diff;
  double ret = 0;
  for(i=0;i<7;i++){
    diff = ang_A.j[i]-ang_B.j[i];
    ret += (diff*diff)*(1+(i<4));  // higher weight to joints near base
  }
  return sqrt(ret);
}


// goto cartesian position
int pa_goto_pos(struct pos_t pos, arm_select LR){

  struct joint_t theta;
  struct joint_t theta_c;
  struct joint_t theta_t;
  struct joint_t *last_angle;
  int k1,k2,k3,i;
  double min_dist = 100000;
  double dist;

  switch(LR){
    
  case LEFT_ARM:
    last_angle = &last_angle_L;    
    theta = inv_kin_L(pos,0.0);
    break;

  case RIGHT_ARM:
    last_angle = &last_angle_R;
    theta = inv_kin_R(pos,0.0);
    break;

  }

  // correct strange definition of directions in physical arm
  /* theta.j[0] *= (-1); */
  /* theta.j[2] *= (-1); */
  /* theta.j[3] *= (-1); */
  /* theta.j[4] *= (-1); */
  /* theta.j[5] *= (-1); */
  /* theta.j[6] *= (-1); */
  
  
  // find solution closest to present
  //PRINT_JOINTS(last_angle_L);
  //PRINT_JOINTS(last_angle_R);
  //PRINT_JOINTS(theta_R);

  // try the 8 different flip combos (j2,j4,j6) can be flipped
  for(k1=0;k1<2;k1++){
    theta_t = theta;
    if(k1){
      theta_t.j[1] *= (-1);
      theta_t.j[0] += PI;
      theta_t.j[2] += PI;
    }

    for(k2=0;k2<2;k2++){
      if(k2){
	theta_t.j[3] *= (-1);
	theta_t.j[2] += PI;
	theta_t.j[4] += PI;
      }

      for(k3=0;k3<2;k3++){
	if(k3){
	  theta_t.j[5] *= (-1);
	  theta_t.j[4] += PI;
	  theta_t.j[6] += PI;
	}

	// keep angles within limits
	for(i=0;i<7;i++){
	  while(theta_t.j[i]>PI){
	    theta_t.j[i] -= (2*PI);
	  }
	  while(theta_t.j[i] < (-PI)){
	    theta_t.j[i] += (2*PI);
	  }
	}

	dist = comp_angles(*last_angle,theta_t);
	if(dist<min_dist){
	  theta_c = theta_t;
	  min_dist = dist;
	  
	}
      }
    }
  }

  //PRINT_JOINTS(theta_L_c);
  //PRINT_JOINTS(theta_R_c);

  return pa_goto_angle(theta_c, LR);
}


int pa_goto_vel(struct joint_t vel, arm_select LR){

  int ret = 0;
  int i;
  int res;
  unsigned int dlc;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  float fvel, fangle;
  struct timespec sleep_time;
  struct joint_t vel_c;

  struct joint_t *last_angle;
  struct module *dumbo_cube;
  int *gripper_exists;
  
  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    last_angle = &last_angle_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    last_angle = &last_angle_R;
    gripper_exists = &gripper_exists_R;
    break;
  }

  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 1000000; // 1 ms


  // saturate the velocities
  for(i=0;i<7;i++){
    if(vel.j[i]> MAX_VEL){
      vel.j[i] = MAX_VEL;
    }
    else if(vel.j[i]<-MAX_VEL){
      vel.j[i] = -MAX_VEL;
    }
  }

  // correct joint angle directions
  // according to the modules' coordinate systems
  vel_c = pa_correct_theta(vel);
  msg[0] = PC_COMMAND_SET_MOTION;
  msg[1] = PC_MOTION_FVEL_ACK;

  for(i=0;i<7;i++){
    // in order to not overload the CAN bus, we sleep between sends:
    nanosleep(&sleep_time,NULL);
    // left arm
    fvel = (float)vel_c.j[i];
    dlc = set_data_float(msg, &fvel);
    pc_send_command_to_module(dumbo_cube[i], msg, dlc);

    if((res=pc_listen_for_response(dumbo_cube[i].handle, &ret_msg))!=(i+1)){
      ret = -1;
      //printf("[%d:%d]",i,res);
    }else{
      get_data_float(ret_msg,&fangle);
      last_angle->j[i] = (double)fangle;
    }
  }

  // correct joint angle directions
  // according to the modules' coordinate systems
  *last_angle = pa_correct_theta(*last_angle);

  // clear CAN stack:
  if(ret!=0){
    pc_listen_for_response(dumbo_cube[0].handle, &ret_msg);
  }

  if(ret !=0){
    ret = -1;
    printf("goto_vel ret: %d\n",ret);
  }

  return ret;
}


int pa_ramp_ended(arm_select LR){

  int i;
  unsigned long int state;
  struct module *dumbo_cube;

  
  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    break;
  }

  for(i=0;i<7;i++){
    
    state = pc_get_status(dumbo_cube[i]);

    if(!(state & PC_STATE_RAMP_END)) return -1;

  }

  return 0;
}


int pa_get_angle(struct joint_t *theta,
		 arm_select LR){
	      
  int ret = 0;
  int i;
  float cur_pos[8];
  unsigned char msg[8];
  unsigned char ret_msg[8];

  struct module *dumbo_cube;
  struct joint_t *last_angle;
  int *gripper_exists;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    last_angle = &last_angle_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    last_angle = &last_angle_R;
    gripper_exists = &gripper_exists_R;
    break;
  }
    
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_ACT_POS;
  
  for(i=0;i<7;i++){
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    get_data_float(ret_msg,&(cur_pos[i]));
    theta->j[i] = (double)cur_pos[i];
    last_angle->j[i] = (double)cur_pos[i];
  }
  
  
  if(*gripper_exists){
    i=7;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    get_data_float(ret_msg,&(cur_pos[i]));
    theta->gripper = (double)cur_pos[i];
    last_angle->gripper = (double)cur_pos[i];
  }
  
  // correct joint angle directions
  // according to the modules' coordinate systems
  *last_angle = pa_correct_theta(*last_angle);
  *theta = pa_correct_theta(*theta);
  
  //*pos = pc_dumbo_forward_kinematics(*theta);
  
  return ret;
}


int pa_get_last_angle(struct joint_t *theta, arm_select LR){

  int ret = 0;
  struct joint_t *last_angle;

  switch(LR){
  case LEFT_ARM:
    last_angle = &last_angle_L;
    break;

  case RIGHT_ARM:
    last_angle = &last_angle_R;
    break;
  }

  *theta = *last_angle;

  return ret;

}



int pa_get_vel( struct joint_t *vel,
		arm_select LR){
  
  int ret = 0;
  int i;
  float cur_vel[8];
  unsigned char msg[8];
  unsigned char ret_msg[8];

  struct module *dumbo_cube;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    break;
  }
    
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_ACT_VEL;
  
  for(i=0;i<7;i++){
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    get_data_float(ret_msg,&(cur_vel[i]));
    vel->j[i] = (double)cur_vel[i];
  }
 
  // correct joint angle directions
  // according to the modules' coordinate systems
  *vel = pa_correct_theta(*vel);
  

  return ret;
}


int pa_get_target_vel(struct joint_t *target_vel, arm_select LR){

  int ret = 0;
  int i;
  float t_vel[8];
  unsigned char msg[8];
  unsigned char ret_msg[8];

  struct module *dumbo_cube;
  int *gripper_exists;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    break;
  }
   
  msg[0] = PC_COMMAND_GET_EXTENDED;
  msg[1] = PC_PARA_TARGET_VEL;
  
  for(i=0;i<7;i++){
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    get_data_float(ret_msg,&(t_vel[i]));
    target_vel->j[i] = (double)t_vel[i];
  }
  
  
  if(*gripper_exists){
    i=7;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    get_data_float(ret_msg,&(t_vel[i]));
    target_vel->gripper = (double)t_vel[i];
  }
  
  // correct joint angle directions
  // according to the modules' coordinate systems
  /* *target_vel = pa_correct_theta(*target_vel); */
  
  //*pos = pc_dumbo_forward_kinematics(*theta);
  
  return ret;
}



int pa_set_target_vel(struct joint_t target_vel, arm_select LR){

  int ret = 0;
  int i;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  unsigned int dlc;
  float fvalue;

  struct module *dumbo_cube;
  int *gripper_exists;
  float *vel_limit;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    vel_limit = vel_limit_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    vel_limit = vel_limit_R;
    break;
  }
   
  //set target vel
  msg[0]=PC_COMMAND_SET_EXTENDED;
  msg[1]=PC_PARA_TARGET_VEL;

  for(i=0;i<7;i++){
    // if the target velocity is within the limits
    if((target_vel.j[i]>0) && (target_vel.j[i]<(double)vel_limit[i])){
      
      fvalue = (float)(target_vel.j[i]);//20*PI/180; // a fast value, 20 deg/s
      dlc = set_data_float(msg,&fvalue);
      pc_send_command_to_module(dumbo_cube[i], msg, dlc);
      if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)){
	ret = -1;
	printf("set target vel error: cube %d\n",i);
      }
      
    }
  }

  if(*gripper_exists){
    i=7;
    // if the target velocity is within the limits
    if((target_vel.gripper>0) && (target_vel.gripper<(double)vel_limit[i])){
      
      fvalue = (float)(target_vel.gripper);//20*PI/180; // a fast value, 20 deg/s
      dlc = set_data_float(msg,&fvalue);
      pc_send_command_to_module(dumbo_cube[i], msg, dlc);
      if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)){
	ret = -1;
	printf("set target vel error: cube %d\n",i);
      }
      
    }
  }
  
  return ret;
}



int pa_get_target_acc(struct joint_t *target_acc, arm_select LR){

  int ret = 0;
  int i;
  float t_acc[8];
  unsigned char msg[8];
  unsigned char ret_msg[8];

  struct module *dumbo_cube;
  int *gripper_exists;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    break;
  }
   
  // first read max acc, then target acc

  msg[0] = PC_COMMAND_GET_EXTENDED;
  
  for(i=0;i<7;i++){
    msg[1] = PC_PARA_MAX_ACC;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;

    msg[1] = PC_PARA_TARGET_ACC;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;

    get_data_float(ret_msg,&(t_acc[i]));
    target_acc->j[i] = (double)t_acc[i];
  }
  
  
  if(*gripper_exists){
    i=7;
    msg[1] = PC_PARA_MAX_ACC;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;

    msg[1] = PC_PARA_TARGET_ACC;
    pc_request_value_from_module(dumbo_cube[i], msg);
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;

    get_data_float(ret_msg,&(t_acc[i]));
    target_acc->gripper = (double)t_acc[i];
  }
  
  
  
  
  return ret;
}



int pa_set_target_acc(struct joint_t target_acc, arm_select LR){

  int ret = 0;
  int i;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  unsigned int dlc;
  float fvalue;

  struct module *dumbo_cube;
  int *gripper_exists;
  float *acc_limit;

  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    acc_limit = acc_limit_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    acc_limit = acc_limit_R;
    break;
  }
   
  //set target acc
  msg[0]=PC_COMMAND_SET_EXTENDED;
  msg[1]=PC_PARA_TARGET_ACC;

  for(i=0;i<7;i++){
    // if the target acc is within the limits
    if((target_acc.j[i]>0) && (target_acc.j[i]<acc_limit[i])){
      
      fvalue = (float)(target_acc.j[i]);//20*PI/180; // a fast value, 20 deg/s
      dlc = set_data_float(msg,&fvalue);
      pc_send_command_to_module(dumbo_cube[i], msg, dlc);
      if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)){
	ret = -1;
	printf("set acc lim error: cube R%d\n",i);
      }
      
    }
  }

  if(*gripper_exists){
    i=7;
    // if the target acc is within the limits
    if((target_acc.gripper>0) && (target_acc.gripper<acc_limit[i])){
      
      fvalue = (float)(target_acc.gripper);//20*PI/180; // a fast value, 20 deg/s
      dlc = set_data_float(msg,&fvalue);
      pc_send_command_to_module(dumbo_cube[i], msg, dlc);
      if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)){
	ret = -1;
	printf("set acc lim error: cube R%d\n",i);
      }
      
    }
  }
  
  return ret;
}



struct pos_t pa_forw_kin(struct joint_t theta, arm_select LR){

  struct pos_t pos;

  switch(LR){
    
  case LEFT_ARM:
    pos = fwd_kin_L(theta, 8);
    break;

  case RIGHT_ARM:
    pos = fwd_kin_R(theta, 8);
    break;
  
  }

  return pos;
}


int pa_halt(arm_select LR){
  int ret = 0;
  int i;
  unsigned char ret_msg[8];

  int *gripper_exists;
  struct module *dumbo_cube;

  switch(LR){

  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    gripper_exists = &gripper_exists_R;
    break;

  }

  for(i=0; i<(7+(*gripper_exists)); i++){
    
    pc_halt_module(dumbo_cube[i]);
    ret_msg[0] = 0;
    if(pc_listen_for_response(dumbo_cube[i].handle,&ret_msg)!=(i+1)) ret = -1;
    if(ret_msg[0]!=PC_COMMAND_HALT){
      ret += -1;
    }

  }


  if(ret !=0) ret = -1;
  return ret;
}

int pa_stop(arm_select LR){

  int ret, i;
  struct joint_t zero_vel;
  int *gripper_exists;

  switch(LR){
  case LEFT_ARM:
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    gripper_exists = &gripper_exists_R;
    break;
  }
  
  for(i=0;i<7;i++){
    zero_vel.j[i] = 0;
  }

  if(*gripper_exists==1){
    zero_vel.gripper = 0;
  }

  ret = pa_goto_vel(zero_vel, LR);

  return ret;
}


int pa_park(arm_select LR){

  int ret = 0;

  //**** have to implement parking procedure

  return ret;
}

int pa_park_L(void){

  int ret = 0;
  //*** have to implement parking procedure

  return ret;
}

int pa_park_R(void){

  int ret = 0;
  //**** have to implement parking procedure

  return ret;
}

struct joint_t pa_correct_theta(struct joint_t theta){

  struct joint_t theta_c = theta;
  int i;

  theta_c.j[0] *= -1;
  theta_c.j[2] *= -1;
  theta_c.j[3] *= -1;
  theta_c.j[4] *= -1;
  theta_c.j[5] *= -1;
  theta_c.j[6] *= -1;

  // keep joint angles within [-pi,pi]
  for(i=0;i<7;i++){
    while(theta_c.j[i]>PI){
      theta_c.j[i] -= (2*PI);
    }
    while(theta_c.j[i] < (-PI)){
      theta_c.j[i] += (2*PI);
    }
  }

  return theta_c;
}


/* struct FT_t pa_get_FT(){ */

/*   int ret_val,i,j; */
/*   signed short int s_out_short_int[7]; */
/*   signed short int SG[6]; */
/*   struct FT_t ft; */

/*   ret_val=get_SG_data(h[sensorChannel],true, &s_out_short_int[0]); */
  
/*   // For some reason, the SG's are in strange order. this fixes: */
/*   SG[0] = s_out_short_int[1]; */
/*   SG[1] = s_out_short_int[4]; */
/*   SG[2] = s_out_short_int[2]; */
/*   SG[3] = s_out_short_int[5]; */
/*   SG[4] = s_out_short_int[3]; */
/*   SG[5] = s_out_short_int[6]; */

/*   // Calculate F/T via cal matrix: */
/*   for(i=0;i<6;i++){ */
/*     ft.f[i] = 0; */
/*     for(j=0;j<6;j++){ */
/*       ft.f[i] += SG[j]* */
/* 	calibration_matrix[i][j]; */
/*     } */
/*     ft.f[i] += bias_val[i]; */
/*   }				 */
/*   return ft; */
/* } */



int pa_col_detect(struct joint_t theta, arm_select LR){

  switch(LR){
  case LEFT_ARM:
    return(col_detect_L(theta));
    break;
      
  case RIGHT_ARM:
    return(col_detect_R(theta));
    break;
  }

  return 0;

}


struct joint_t pa_col_sat_vel(struct joint_t vel, struct joint_t theta, 
			      struct joint_t target_acc, double vel_control_freq, 
			      arm_select LR){

  struct joint_t sat_vel;

  switch(LR){
  case LEFT_ARM:
    sat_vel = col_sat_vel_L(vel, theta, target_acc, vel_control_freq);
    break;

  case RIGHT_ARM: 
    sat_vel = col_sat_vel_R(vel, theta, target_acc, vel_control_freq);
    break;
  }

  return sat_vel;
}


int pa_col_check_vel(struct joint_t vel, struct joint_t theta, arm_select LR){
  
  int ret;

  switch(LR){

  case LEFT_ARM:
    ret = col_check_vel_L(vel, theta);
    break;

  case RIGHT_ARM:
    ret = col_check_vel_R(vel, theta);
    break;

  }

  return ret;

}



int pa_goto_vel_DO(struct joint_t vel, arm_select LR){

  int ret = 0;
  int i;
  int res;
  unsigned int dlc;
  unsigned char msg[8];
  unsigned char ret_msg[8];
  float fvel, fangle;
  struct timespec sleep_time;
  struct joint_t vel_c;

  struct joint_t *last_angle;
  struct module *dumbo_cube;
  int *gripper_exists;
  
  switch(LR){
  case LEFT_ARM:
    dumbo_cube = dumbo_cube_L;
    last_angle = &last_angle_L;
    gripper_exists = &gripper_exists_L;
    break;

  case RIGHT_ARM:
    dumbo_cube = dumbo_cube_R;
    last_angle = &last_angle_R;
    gripper_exists = &gripper_exists_R;
    break;
  }

  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 1000000; // 1 ms


  // saturate the velocities
  for(i=0;i<7;i++){
    if(vel.j[i]> MAX_VEL){
      vel.j[i] = MAX_VEL;
    }
    else if(vel.j[i]<-MAX_VEL){
      vel.j[i] = -MAX_VEL;
    }
  }

  // correct joint angle directions
  // according to the modules' coordinate systems
  vel_c = pa_correct_theta(vel);
  msg[0] = PC_COMMAND_SET_MOTION;
  msg[1] = PC_MOTION_FVEL_ACK;


  // send vel motion to 2nd joint
  i = 1;
  // in order to not overload the CAN bus, we sleep between sends:
  nanosleep(&sleep_time,NULL);
  // left arm
  fvel = (float)vel_c.j[i];
  dlc = set_data_float(msg, &fvel);
  pc_send_command_to_module(dumbo_cube[i], msg, dlc);

  if((res=pc_listen_for_response(dumbo_cube[i].handle, &ret_msg))!=(i+1)){
    ret = -1;
    //printf("[%d:%d]",i,res);
  }else{
    get_data_float(ret_msg,&fangle);
    last_angle->j[i] = (double)fangle;
  }

  // send vel motion to 5th joint
  i = 4;
  // in order to not overload the CAN bus, we sleep between sends:
  nanosleep(&sleep_time,NULL);
  // left arm
  fvel = (float)vel_c.j[i];
  dlc = set_data_float(msg, &fvel);
  pc_send_command_to_module(dumbo_cube[i], msg, dlc);

  if((res=pc_listen_for_response(dumbo_cube[i].handle, &ret_msg))!=(i+1)){
    ret = -1;
    //printf("[%d:%d]",i,res);
  }else{
    get_data_float(ret_msg,&fangle);
    last_angle->j[i] = (double)fangle;
  }
  


  // correct joint angle directions
  // according to the modules' coordinate systems
  last_angle->j[4]*= -1;

  // clear CAN stack:
  if(ret!=0){
    pc_listen_for_response(dumbo_cube[0].handle, &ret_msg);
  }

  if(ret !=0){
    ret = -1;
  }

  return ret;
}



void pa_kill_watchdogs(pthread_t watchdog_t){

  pc_kill_watchdog(watchdog_t);

}
