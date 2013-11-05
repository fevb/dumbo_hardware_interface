/*
 *  ComunicationCanx.h
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

# ifndef Comunication_canx_h
# define Comunication_canx_h

# include <kvaser_canlib/canlib.h>

//typedef enum {false, true} bool;
//define maximum waiting time (ms)
# define CB_max_wait_time 500


#define CB_Shunk_ReadData 		0x200
#define CB_Shunk_ReadMatrix 		0x202
#define CB_Shunk_ReadSN 		0x205
#define  CB_Shunk_Calibration 		0x206
#define  CB_Shunk_ReadCountsPerUnit 	0x207
#define  CB_Shunk_ReadUnitCodes		0x208
#define  CB_Shunk_Reset 		0x20c
#define  CB_Shunk_SetBaseIdentifier 	0x20d
#define  CB_Shunk_SetBaudRate 		0x20e
#define  CB_Shunk_ReadFirmwareVersion 	0x20f
// define parameters to send
#define CB_ReadMatrix_Fx 	0
#define CB_ReadMatrix_Fy 	1
#define CB_ReadMatrix_Fz 	2
#define CB_ReadMatrix_Tx 	3
#define CB_ReadMatrix_Ty 	4
#define CB_ReadMatrix_Tz 	5


// define the possible types of data to receive
#define CB_Receive_ReadData1 		0x200
#define CB_Receive_ReadData2 		0x201
#define CB_Receive_ReadMatrix1 		0x202
#define CB_Receive_ReadMatrix2 		0x203
#define CB_Receive_ReadMatrix3 		0x204
#define CB_Receive_ReadSN 		0x205
#define CB_Receive_Calibration		0x206
#define CB_Receive_ReadCountsPerUnit 	0x207
#define CB_Receive_ReadUnitCodes 	0x208
#define CB_Receive_SetBaseIdentifier	0x20d
#define CB_Receive_SetBaudRate		0x20e
#define CB_Receive_ReadFirmwareVersion	0x20f


struct Firmware_version{
	char major;
	char minor;
	short int build_number;
};


int canSensor_Send(canHandle h,long types,char *i_additional_info, bool wait_send_message);
int canSensor_Recive(canHandle h,long type,void *pt_msg,bool wait_recive_message,unsigned long *time_stamp);
int find_error_StatusCode(short int *num_dec);
char convert_base_identifier(int bits[8]);


# endif /*Comunication_canx_h*/
