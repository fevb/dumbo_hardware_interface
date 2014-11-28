/*
 *  ft_sensor_function
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

# include <stdio.h>
# include <dumbo_force_torque_sensor/ft_sensor_function.h>

/*
 -*float [6]: output vector of force/torque 
 -signed short int SG_Data [7]: Strain Gage with StatusCode 
 -float Transducer_Matrix [6][6]: Matrix of Calibration
 -signed short int bias [6]: bias vector (could be NULL)
 */
int Force_Torque_Data(double *out_FT,signed short int SG_Data[7],float Transducer_Matrix[6][6],float bias[6]){ //output *double[6]
	int i,k;
	float temp;
	for (i=0;i<6;i++) {
		temp=0;
		for(k=0;k<6;k++) temp=temp+Transducer_Matrix[i][k]*(SG_Data[k+1]-bias[k]);
		*(out_FT+i)=temp;
		} 
	return (0);
}




int get_SG_data(canHandle canbus1,bool wait, signed short int *s_out_short_int){ //input/output: *signed short int[7]
	int ret_val=canOK;
	ret_val=canSensor_Send(canbus1,CB_Shunk_ReadData,NULL, wait);
	if (ret_val!=canOK) return(-1);			
	ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadData,(void*)s_out_short_int,wait,NULL);
	if (ret_val!=canOK) return(-1);
	endian_short_int_conversion(s_out_short_int,7);
	return (canOK);
}

int request_SG_data(canHandle canbus1, bool wait)
{
    int ret_val=canOK;
    ret_val=canSensor_Send(canbus1,CB_Shunk_ReadData,NULL, wait);
    if (ret_val!=canOK) return(-1);
}

int read_SG_data(canHandle canbus1, bool wait, signed short *s_out_short_int)
{
    int ret_val=canOK;
    ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadData,(void*)s_out_short_int,wait,NULL);
    if (ret_val!=canOK) return(-1);
    endian_short_int_conversion(s_out_short_int,7);
    return (canOK);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////						
int get_Transducer_CalMatrix(canHandle h,float *out){ //input/output: *float[6][6]
// ALREADY DIVIDED BY COUNTS PER UNITS!!!
	int f_err_out=0;
	char row=0;
	int CpU[2];
	f_err_out=get_CountsPerUnit(h,true,&CpU[0]);
	if (f_err_out==canOK){
		for (row=0;row<6;row++,out=out+6){
			f_err_out=canSensor_Send(h,CB_Shunk_ReadMatrix,&row,true);
			if (f_err_out!=canOK) f_err_out=-1;
			f_err_out=canSensor_Recive(h,CB_Shunk_ReadMatrix,out,true,NULL);
			endian_float_conversion(out,6) ;
			*out=*out/CpU[0];
			*(out+1)=*(out+1)/CpU[0];
			*(out+2)=*(out+2)/CpU[0];
			*(out+3)=*(out+3)/CpU[1];
			*(out+4)=*(out+4)/CpU[1];
			*(out+5)=*(out+5)/CpU[1];
			//check error
			if (f_err_out==-1) row=6;
			}
		}
	else f_err_out=-1;
	return (f_err_out);
}

					
int get_Serial_Number(canHandle canbus1,bool wait, char *out_char_eight){ //input/output: *char[8]			
	int ret_val=canOK;
	ret_val=canSensor_Send(canbus1,CB_Shunk_ReadSN,NULL, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadSN,(void*)out_char_eight,wait,NULL);
	if (ret_val!=canOK) return(-1);
	return(0);
}
			
			
			
			
			
			
int set_Calibration(canHandle canbus1,bool wait, int n_calib,char *out_char){ //input/output: *char			
	int ret_val=canOK;
	if ((n_calib>15)||(n_calib<0)) {
			printf("Bad number of calibration\n");
			return(-1);
		}
	char calib=(char)n_calib;
	ret_val=canSensor_Send(canbus1,CB_Shunk_Calibration,&calib, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_Calibration,(void*)out_char,wait,NULL);
	if (ret_val!=canOK) return(-1);
	return(0);
}	
				
				
				
				
				
int get_CountsPerUnit(canHandle canbus1,bool wait,int *out_int_two){	//input/output: *float[2]			
	int ret_val=canOK;
	ret_val=canSensor_Send(canbus1,CB_Shunk_ReadCountsPerUnit,NULL, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadCountsPerUnit,(void*)out_int_two,wait,NULL);
	if (ret_val!=canOK) return(-1);
	endian_int_Conversion(out_int_two,2);
	return(0);
}	
				
				
				
				
int get_UnitCodes(canHandle canbus1,bool wait,char *out_char_two){	//input/output: *char[2]			
	int ret_val=canOK;				
	ret_val=canSensor_Send(canbus1,CB_Shunk_ReadUnitCodes,NULL, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadUnitCodes,(void*)out_char_two,wait,NULL);
	if (ret_val!=canOK) return(-1);
	return(0);
}	
				
				
				
int get_Reset(canHandle canbus1){	// no input-output			
	int ret_val=canOK;								
	ret_val=canSensor_Send(canbus1,CB_Shunk_Reset,NULL, true);
	if (ret_val!=canOK) return(-1);
	else{
		printf("RESET OK!\n");
		return(0);
		}
}	
				
				
int set_BaseIdentifier(canHandle canbus1,char *out_char){	//input/output: *char			
	int ret_val=canOK;
	ret_val=canSensor_Send(canbus1,CB_Shunk_SetBaseIdentifier,out_char, true);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_SetBaseIdentifier,NULL,true,NULL);
	if (ret_val!=canOK) return(-1);
	else{
		printf("changed at the next powerup\n");
		return(0);
		}
}	
			
			

int set_BaudRate(canHandle canbus1,bool wait,int divisor){ //input/output: *char
	int ret_val=canOK;
	divisor=divisor-1;
	char c_divisor=(char)divisor;
	ret_val=canSensor_Send(canbus1,CB_Shunk_SetBaudRate,&c_divisor, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_SetBaudRate,NULL,wait,NULL);
	if (ret_val!=canOK) return(-1);
	else{
		printf("changed at the next powerup\n");
		return(0);
		}
}		
				
				
				
				
				
int get_Firmware_version(canHandle canbus1,bool wait,struct Firmware_version *out_struct){	//input/output: *struct Firmware_version out_struct;
	int ret_val=canOK;			
	ret_val=canSensor_Send(canbus1,CB_Shunk_ReadFirmwareVersion,NULL, wait);
	if (ret_val!=canOK) return(-1);
	ret_val=canSensor_Recive(canbus1,CB_Shunk_ReadFirmwareVersion,(void*)out_struct,wait,NULL);
	if (ret_val!=canOK) return(-1);
	endian_short_int_conversion(&(out_struct->build_number),1) ;
	return(0);
}
				
				
				
void endian_short_int_conversion(signed short int *word,int lenght) {
   int i;
	for (i=0;i<lenght;i++,word++)
	*word=((*word>>8)&0x00FF) | ((*word<<8)&0xFF00)  ;
}
	
			
void endian_float_conversion(float *inFloat, int lenght ){
	int i;
	for (i=0;i<lenght;i++,inFloat++){
		float retVal;
		char *floatToConvert = ( char* ) inFloat;
		char *returnFloat = ( char* ) & retVal;

		// swap the bytes into a temporary buffer
		returnFloat[0] = floatToConvert[3];
		returnFloat[1] = floatToConvert[2];
		returnFloat[2] = floatToConvert[1];
		returnFloat[3] = floatToConvert[0];
		*inFloat=retVal;
		}
}
				
				
void endian_int_Conversion(int *dword,int lenght) {
	int i;
	for (i=0;i<lenght;i++,dword++) *dword=((*dword>>24)&0x000000FF) | ((*dword>>8)&0x0000FF00) | ((*dword<<8)&0x00FF0000) | ((*dword<<24)&0xFF000000);
}
				
	
				
				
