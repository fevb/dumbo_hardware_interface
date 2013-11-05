/*
 *  ComunicationCanx.cpp
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

// invoke with -lm
# include <stdio.h>
# include <stdlib.h>
# include <math.h>
# include <kvaser_canlib/canlib.h>
# include <dumbo_force_torque_sensor/ComunicationCanx.h>
# include <string.h>

/* shunk_send: send a message to the shunk:
- canHandle 
- types: type of message: type are define in ComunicationCanx.h (example: send and read data, asking for matrix...)
- *i_additional_info: necessary for some request (ex: read matrix need the row): it's the message send
- wait_send_message: bool types: if 1, the program wait until the message is send else it put the message in the out buffer and go on
*/
int canSensor_Send(canHandle h,long types,char *i_additional_info, bool wait_send_message){

	canStatus retStatus;
	//check on the additional info
	if (i_additional_info==NULL && (types== CB_Shunk_ReadMatrix || types== CB_Shunk_Calibration || types== CB_Shunk_SetBaseIdentifier || types== CB_Shunk_SetBaudRate))
		{
		printf("Error calling shunk_send: bad value\n");
		return(-1);
		}
	//send data
	if (wait_send_message==false){
		if (types==CB_Shunk_ReadData)			retStatus = canWrite(h, types, NULL, 0, 0);
		else if (types==CB_Shunk_ReadMatrix)		retStatus = canWrite(h, types, i_additional_info, 1, 0);
		else if (types==CB_Shunk_ReadSN)		retStatus = canWrite(h, types, NULL, 0, 0);
		else if (types==CB_Shunk_Calibration)		retStatus = canWrite(h, types, i_additional_info, 1, 0);
		else if (types==CB_Shunk_ReadCountsPerUnit)	retStatus = canWrite(h, types, NULL, 0, 0);
		else if (types==CB_Shunk_ReadUnitCodes)		retStatus = canWrite(h, types, NULL, 0, 0);
		else if (types==CB_Shunk_Reset)			retStatus = canWrite(h, types, NULL, 0, 0);
		else if (types==CB_Shunk_SetBaseIdentifier)	retStatus = canWrite(h, types, i_additional_info, 1, 0);
		else if (types==CB_Shunk_SetBaudRate)		retStatus = canWrite(h, types, i_additional_info, 1, 0);
		else if (types==CB_Shunk_ReadFirmwareVersion)	retStatus = canWrite(h, types, NULL, 0, 0);
		else { 
			printf("on function 'canSensor_send': no type matching for request data\n");
			exit(1);
			}
	}	
	else {
		if (types==CB_Shunk_ReadData)			retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else if (types==CB_Shunk_ReadMatrix) 		retStatus = canWriteWait(h, types, i_additional_info, 1, 0,CB_max_wait_time);
		else if (types==CB_Shunk_ReadSN) 		retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else if (types==CB_Shunk_Calibration) 		retStatus = canWriteWait(h, types, i_additional_info, 1, 0,CB_max_wait_time);
		else if (types==CB_Shunk_ReadCountsPerUnit)	retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else if (types==CB_Shunk_ReadUnitCodes)		retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else if (types==CB_Shunk_Reset)			retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else if (types==CB_Shunk_SetBaseIdentifier)	retStatus = canWriteWait(h, types, i_additional_info, 1, 0,CB_max_wait_time);
		else if (types==CB_Shunk_SetBaudRate)		retStatus = canWriteWait(h, types, i_additional_info, 1, 0,CB_max_wait_time);
		else if (types==CB_Shunk_ReadFirmwareVersion)	retStatus = canWriteWait(h, types, NULL, 0, 0,CB_max_wait_time);
		else { 
			printf("on function 'canSensor_send': no type matching for request data\n");
			exit(1);
			}		
	}
	// check error on sending		
	if (retStatus <0 ){
		char err_msg[60];
		canGetErrorText(retStatus, err_msg, sizeof(err_msg));
		printf( "on function 'canSensor_send': Failed, can't write: (%s)\n", err_msg);
		return (-1);
	}
	else return (0);
}


/* shunk_recive_ReadData: function for recive the response of CB_Shunk_ReadData: wait until it's arrived
- canHandle h
- type of answer: same of type of message for shunk_send 
- *pt pointer at array for output data: need array, whith right type and dimension (no check inside!!) 
- wait_recive_message: if true wait until a specific message is arrived
the function return:
 0 OK
 1 NO Message available (case with wait_recive_message=true)
 -1 ERROR
*/
int canSensor_Recive(canHandle h,long type,void *pt_msg,bool wait_recive_message,unsigned long *time_stamp)
{
	canStatus retStatus1 = canOK,retStatus2=canOK ,retStatus3=canOK;
	//int i;
	unsigned int dlc;
	if (wait_recive_message==false){
		if (type==CB_Shunk_ReadData){ //output type: short int array[7]
			short int *pt_msg_in=(short int*)pt_msg;
			retStatus1 = canRead(h,&type,pt_msg,&dlc,NULL,time_stamp);
			type=CB_Receive_ReadData2;
			pt_msg=pt_msg+dlc;
			retStatus2 = canRead(h,&type,pt_msg,&dlc,NULL,time_stamp);
			//controll error on statusCode
			*pt_msg_in=((*pt_msg_in>>8)&0x00FF) | ((*pt_msg_in<<8)&0xFF00)  ;
			
			find_error_StatusCode(pt_msg_in);
			}
		else if (type==CB_Shunk_ReadMatrix){ //output type: float array[6]
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			type=CB_Receive_ReadMatrix2;
			pt_msg=pt_msg+dlc;
			retStatus2 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			type=CB_Receive_ReadMatrix3;
			pt_msg=pt_msg+dlc;
			retStatus3 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
	
	 	else if (type==CB_Shunk_ReadSN){ //output type: char array[8]
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
		else if (type==CB_Shunk_Calibration){ //output type: char 
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
		else if (type==CB_Shunk_ReadCountsPerUnit){ //output type: float array[2]
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
		else if (type==CB_Shunk_ReadUnitCodes){ //output type: char array[2]
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
		else if (type==CB_Shunk_Reset){ //error!! no response
			printf("no answer with 'reset' in function: 'shunk_recive_ReadData'\n");
			exit(1);
			}
		else if (type==CB_Shunk_SetBaseIdentifier){ //no output
			retStatus1 = canRead (h,&type,NULL,NULL,NULL,time_stamp);
			if (retStatus1 == canOK ) printf("changed base identifier: will take effect at the next powerup\n");
			}
		else if (type==CB_Shunk_SetBaudRate){ //no output
			retStatus1 = canRead (h,&type,NULL,NULL,NULL,time_stamp);
			if (retStatus1 == canOK ) printf("changed baud rate: will take effect at the next powerup\n");
			}	
		else if (type==CB_Shunk_ReadFirmwareVersion){ // output type: struct Firmware_version
			retStatus1 = canRead (h,&type,pt_msg,&dlc,NULL,time_stamp);
			}
		else	{ 
			printf("no type matching in function: 'shunk_recive_ReadData' \n");
			exit(1);
			}
		if ((retStatus1 !=canOK )|| (retStatus2 !=canOK )||(retStatus3 !=canOK )) 
			{
			     char err_msg[100];
			     canGetErrorText(retStatus1, err_msg, sizeof(err_msg));
			     printf( "on function 'canSensor_Recive'::Failed! can't read: (%s)\n",err_msg);
			     return(-1);
			}
		else if ((retStatus1 == canERR_NOMSG )|| (retStatus2 == canERR_NOMSG )||(retStatus3 == canERR_NOMSG )) return (1);
		else	return (0);
		}
	else {
		if (type==CB_Shunk_ReadData){ //output type: short int array[7]
			void *pt_msg_in=pt_msg;
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			type=CB_Receive_ReadData2;
			pt_msg=pt_msg+dlc;
			retStatus2 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			//controll error on statusCode
			find_error_StatusCode((short int*)pt_msg_in);
			}
		else if (type==CB_Shunk_ReadMatrix){ //output type: float array[6]
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			type=CB_Receive_ReadMatrix2;
			pt_msg=pt_msg+dlc;
			retStatus2 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			type=CB_Receive_ReadMatrix3;
			pt_msg=pt_msg+dlc;
			retStatus3 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else if (type==CB_Shunk_ReadSN){ //output type: char array[8]
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else if (type==CB_Shunk_Calibration){ //output type: char 
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else if (type==CB_Shunk_ReadCountsPerUnit){ //output type: float array[2]
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else if (type==CB_Shunk_ReadUnitCodes){ //output type: char array[2]
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else if (type==CB_Shunk_Reset){ //error!! no response
			printf("no answer with 'reset' in function: 'shunk_recive_ReadData'\n");
			exit(1);
			}
		else if (type==CB_Shunk_SetBaseIdentifier){ //no output
			retStatus1 = canReadWait (h,&type,NULL,NULL,NULL,time_stamp,CB_max_wait_time);
			if (retStatus1 == canOK ) printf("changed base identifier: will take effect at the next powerup\n");
			}
		else if (type==CB_Shunk_SetBaudRate){ //no output
			retStatus1 = canReadWait (h,&type,NULL,NULL,NULL,time_stamp,CB_max_wait_time);
			if (retStatus1 == canOK ) printf("changed baud rate: will take effect at the next powerup\n");
			}	
		else if (type==CB_Shunk_ReadFirmwareVersion){ // output type: struct Firmware_version
			retStatus1 = canReadWait (h,&type,pt_msg,&dlc,NULL,time_stamp,CB_max_wait_time);
			}
		else	{ 
			printf("no type matching in function: 'canSensor_Recive' \n");
			exit(1);
			}
	if ((retStatus1 !=canOK )|| (retStatus2 !=canOK )||(retStatus3 !=canOK )) 
		{
		     char err_msg[100];
		     canGetErrorText(retStatus1, err_msg, sizeof(err_msg));
		     printf( "Failed! can't read: (%s)\n",err_msg);
		     return(-1);
			}
	else return (0);
	}
}		





/* invoke with pointer to a number that need analized (also returned from 'Read SG Data')...
 if return 0, it's all ok!, else the how many errors that there are */
int find_error_StatusCode(short int *num_dec)
{	
	int n_error=0 ;
	//convert the number recived into a vector of bit
	short int bit,num_bits,out_code[16];
	for (num_bits=0;num_bits<16;num_bits++)
		{		
		bit = *num_dec % 2 ;
		*num_dec = *num_dec/2 ;
		out_code[num_bits]=bit;
		}
	if (out_code[15]!=0) //check if there are any error
	{
		printf("Error on NETCANOEM Status:\n");
		for (num_bits=14;num_bits>=0;num_bits--) 
		{
			n_error++;
			if (out_code[num_bits]!=0) 
				{
					switch(num_bits)
					{
						case 0:
							printf("Error Satus Code: Watchdog Reset\n");
							break;
						case 1:
							printf("Error Satus Code: DAC/ADC check result too high\n");
							break;
						case 2:
							printf("Error Satus Code: DAC/ADC check result too low\n");
							break;
						case 3:
							printf("Error Satus Code: Artificial analog ground out of range\n");
							break;
						case 4:
							printf("Error Satus Code: Power supply too high\n");
							break;
						case 5:
							printf("Error Satus Code: Power supply too low\n");
							break;
						case 6:
							printf("Error Satus Code: Bad active calibration\n");
							break;
						case 7:
							printf("Error Satus Code: EEPROM failure\n");
							break;
						case 8:
							printf("Error Satus Code: Configuration Invalid\n");
							break;
						case 11:
							printf("Error Satus Code: Sensor temperature too high\n");
							break;
						case 12:
							printf("Error Satus Code: Sensor temperature too low \n");
							break;
						case 14:
							printf("Error Satus Code: CAN bus error\n");
							break;
						}
				}
		}
	}
	//else printf("NO ERROR\n");
	return(n_error);
}



/*convert an array[0] with the walue of the sigle bit (array[0] is the lower) into a char*/ 
char convert_base_identifier(int bits[8])
{
	char out;
	int numdec = 0,i;
	for(i = 0; i<sizeof(bits);i++) numdec =+ (bits[i]*pow(2,i));
	out= (char) numdec;
	return (out);
}







