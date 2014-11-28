/*
 *  ft_sensor_function.h
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


# ifndef ft_sensor_function_h
# define ft_sensor_function_h
//# include "ft_sensor_function.h"
# include <dumbo_force_torque_sensor/ComunicationCanx.h>
# include <kvaser_canlib/canlib.h>


int Force_Torque_Data(double *out_FT,signed short int SG_Data[7],float Transducer_Matrix[6][6],float bias[6]); //output *double[6]
int get_SG_data(canHandle canbus1,bool wait, signed short int *s_out_short_int); //input/output: *signed short int[7]

// get_SG_data split in 2 parts: request measurement and read measurement
int request_SG_data(canHandle canbus1, bool wait);
int read_SG_data(canHandle canbus1,bool wait, signed short int *s_out_short_int);

int get_Transducer_CalMatrix(canHandle h,float *out); //input/output: *float[6][6]
int get_Serial_Number(canHandle canbus1,bool wait, char *out_char_eight); //input/output: *char[8]
int set_Calibration(canHandle canbus1,bool wait, int n_calib,char *out_char); //input/output: *char
int get_CountsPerUnit(canHandle canbus1,bool wait,int *out_int_two);	//input/output: *float[2]	
int get_UnitCodes(canHandle canbus1,bool wait,char *out_char_two);	//input/output: *char[2]
int get_Reset(canHandle canbus1);	// no input-output	
int set_BaseIdentifier(canHandle canbus1,char *out_char);	//input/output: *char	
int set_BaudRate(canHandle canbus1,bool wait,int divisor); //input/output: *char
int get_Firmware_version(canHandle canbus1,bool wait,struct Firmware_version *out_struct);	//input/output: *struct Firmware_version




void endian_float_conversion(float *inFloat, int lenght ) ;
void endian_short_int_conversion(signed short int *word,int lenght) ;
void endian_int_Conversion(int *dword,int lenght) ;

# endif




