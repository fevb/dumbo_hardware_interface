/*
  This file defines constants for use with the Amtec PowerCubes
*/

#ifndef _powercube_defines_H
#define _powercube_defines_H

#ifndef MAX_POPEYE_CUBES
#define MAX_POPEYE_CUBES (6)
#endif

/*
  CAN ID's
*/
#define PC_CANID_CMDACK                      0x0a0
#define PC_CANID_CMDGET                      0x0c0
#define PC_CANID_CMDPUT                      0x0e0
#define PC_CANID_CMDALL                      0x100

/*
  Joint adresses
*/
#define PC_MODULE_ID_J1                      0x01
#define PC_MODULE_ID_J2                      0x02
#define PC_MODULE_ID_J3                      0x03
#define PC_MODULE_ID_J4                      0x04
#define PC_MODULE_ID_J5                      0x05
#define PC_MODULE_ID_J6                      0x06

/*
  'Command All' ID's
*/
#define PC_ALL_COMMAND_RESET                     0x00
#define PC_ALL_COMMAND_HOME                      0x01
#define PC_ALL_COMMAND_HALT                      0x02
#define PC_ALL_COMMAND_WATCHDOG_REFRESH          0x07
#define PC_ALL_COMMAND_CHANGE_BAUDRATE           0x09
#define PC_ALL_COMMAND_SAVE_POSITION             0x0e
#define PC_ALL_COMMAND_SYNC_MOTION               0x0f


/*
  Command ID's
*/
#define PC_COMMAND_RESET                     0x00
#define PC_COMMAND_HOME                      0x01
#define PC_COMMAND_HALT                      0x02
#define PC_COMMAND_RECALC_PID_PARAM          0x09
#define PC_COMMAND_SET_EXTENDED              0x08
#define PC_COMMAND_GET_EXTENDED              0x0a
#define PC_COMMAND_SET_MOTION                0x0b
#define PC_COMMAND_SET_I_STEP                0x0d

/*
  Motion ID's
*/
// Floats
#define PC_MOTION_FRAMP_MODE                 4
#define PC_MOTION_FSTEP_MODE                 6
#define PC_MOTION_FVEL_MODE                  7 
#define PC_MOTION_FCUR_MODE                  8
// Ints
#define PC_MOTION_IRAMP_MODE                 9
#define PC_MOTION_ISTEP_MODE                 11
#define PC_MOTION_IVEL_MODE                  12
#define PC_MOTION_ICUR_MODE                  13
// Floats with acknowledge
#define PC_MOTION_FRAMP_ACK                  14
#define PC_MOTION_FSTEP_ACK                  16
#define PC_MOTION_FVEL_ACK                   17 
#define PC_MOTION_FCUR_ACK                   18
// Ints with acknowledge
#define PC_MOTION_IRAMP_ACK                  19
#define PC_MOTION_ISTEP_ACK                  21
#define PC_MOTION_IVEL_ACK                   22
#define PC_MOTION_ICUR_ACK                   23


/*
  Short states in acknowledge
*/
#define PC_ACK_SHORT_NOT_OK                  0x01
#define PC_ACK_SHORT_SWR                     0x02
#define PC_ACK_SHORT_SW1                     0x04
#define PC_ACK_SHORT_SW2                     0x08
#define PC_ACK_SHORT_MOTION                  0x10
#define PC_ACK_SHORT_RAMP_END                0x20
#define PC_ACK_SHORT_INPROGRESS              0x40
#define PC_ACK_SHORT_FULLBUFFER              0x80


/*
  Digital IO states
*/
#define PC_ACK_IO_INBIT0                     0x01
#define PC_ACK_IO_INBIT1                     0x02
#define PC_ACK_IO_INBIT2                     0x04
#define PC_ACK_IO_INBIT3                     0x08
#define PC_ACK_IO_OUTBIT0                    0x10
#define PC_ACK_IO_OUTBIT1                    0x20
#define PC_ACK_IO_OUTBIT2                    0x40
#define PC_ACK_IO_OUTBIT3                    0x80

/*
  PowerCube Parameter ID's
*/
// Defaults
#define DEF_POPEYE_SERIAL_NUMBER	     3255
#define DEF_DUMBO_SERIAL_NUMBER	             40807
#define DEF_SCHUNK_LEFTARM_SERIAL_NUMBER     41261    
#define DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER    41253
#define DEF_SCHUNK_GRIPPER_SERIAL_NUMBER     20278	

#define PC_PARA_DEF_HOME_OFFSET              0x00
#define PC_PARA_DEF_GEAR_RATIO               0x01
#define PC_PARA_DEF_LIN_RATIO                0x02
#define PC_PARA_DEF_MIN_POS                  0x03
#define PC_PARA_DEF_MAX_POS                  0x04
#define PC_PARA_DEF_MAX_DELTA_POS            0x05
#define PC_PARA_DEF_MAX_DELTA_VEL            0x06
#define PC_PARA_DEF_TORQUE_RATIO             0x07
#define PC_PARA_DEF_CUR_RATIO                0x08

#define PC_PARA_DEF_MIN_VEL                  0x09
#define PC_PARA_DEF_MAX_VEL                  0x0A
#define PC_PARA_DEF_MIN_ACC                  0x0B
#define PC_PARA_DEF_MAX_ACC                  0x0C
#define PC_PARA_DEF_MIN_CUR                  0x0D
#define PC_PARA_DEF_MAX_CUR                  0x0E
#define PC_PARA_DEF_HOME_VEL                 0x0F
#define PC_PARA_DEF_HOME_ACC                 0x10


#define PC_PARA_DEF_CUBE_SERIAL              0x1A
#define PC_PARA_DEF_CONFIG                   0x1B
#define PC_PARA_DEF_PULSES_PER_TURN          0x1C
#define PC_PARA_DEF_CUBE_VERSION             0x1D
#define PC_PARA_DEF_SERVICE_INTERVAL         0x1E
#define PC_PARA_DEF_BRAKE_TIME_OUT           0x1F

#define PC_PARA_DEF_ADDRESS                  0x20
#define PC_PARA_DEF_PRIM_BAUD                0x22
#define PC_PARA_DEF_SCND_BAUD                0x23

// Runtime variables
#define PC_PARA_POS_COUNT                    0x24
#define PC_PARA_REF_POS_COUNT                0x25
#define PC_PARA_DIO_SETUP                    0x26
#define PC_PARA_CUBE_STATE                   0x27
#define PC_PARA_TARGET_POS_INC               0x28
#define PC_PARA_TARGET_VEL_INC               0x29
#define PC_PARA_TARGET_ACC_INC               0x2A
#define PC_PARA_STEP_INC                     0x2B
#define PC_PARA_HOME_OFFSET_INC              0x2C

#define PC_PARA_RAW_CUR                      0x35
#define PC_PARA_HOME_TO_ZERO_INC             0x36
#define PC_PARA_CONFIG                       0x39
#define PC_PARA_MOVE_MODE                    0x3A
#define PC_PARA_INC_RATIO                    0x3B
#define PC_PARA_ACT_POS                      0x3C
#define PC_PARA_ACT_POS_                     0x3D
#define PC_PARA_IPOL_POS                     0x3E
#define PC_PARA_DELTA_POS                    0x3F

#define PC_PARA_MAX_DELTA_POS                0x40
#define PC_PARA_ACT_VEL                      0x41
#define PC_PARA_IPOL_VEL                     0x42
#define PC_PARA_MIN_POS                      0x45
#define PC_PARA_MAX_POS                      0x46
#define PC_PARA_MAX_VEL                      0x48
#define PC_PARA_MAX_ACC                      0x4A
#define PC_PARA_MAX_CUR                      0x4C
#define PC_PARA_CUR                          0x4D

#define PC_PARA_TARGET_POS                   0x4E
#define PC_PARA_TARGET_VEL                   0x4F
#define PC_PARA_TARGET_ACC                   0x50
#define PC_PARA_DEF_C0                       0x51
#define PC_PARA_DEF_DAMP                     0x52
#define PC_PARA_DEF_A0                       0x53
#define PC_PARA_ACT_C0                       0x54
#define PC_PARA_ACT_DAMP                     0x55

#define PC_PARA_ACT_A0                       0x56
#define PC_PARA_DEF_BURN_COUNT               0x57
#define PC_PARA_SETUP                        0x58
#define PC_PARA_HOME_OFFSET                  0x59

/*
  Module states  (accessed with parameter ID 0x27)
*/
#define PC_STATE_ERROR                       0x00000001
#define PC_STATE_HOME_OK                     0x00000002
#define PC_STATE_HALTED                      0x00000004
#define PC_STATE_POWERFAULT                  0x00000008
#define PC_STATE_TOW_ERROR                   0x00000010
#define PC_STATE_COMM_ERROR                  0x00000020
#define PC_STATE_SWR                         0x00000040
#define PC_STATE_SW1                         0x00000080

#define PC_STATE_SW2                         0x00000100
#define PC_STATE_BRAKE_ACTIVE                0x00000200
#define PC_STATE_CUR_LIMIT                   0x00000400
#define PC_STATE_MOTION                      0x00000800
#define PC_STATE_RAMP_ACC                    0x00001000
#define PC_STATE_RAMP_STEADY                 0x00002000
#define PC_STATE_RAMP_DEC                    0x00004000
#define PC_STATE_RAMP_END                    0x00008000

#define PC_STATE_IN_PROGRESS                 0x00010000
#define PC_STATE_FULLBUFFER                  0x00020000
#define PC_STATE_POW_VOLT_ERROR              0x00040000
#define PC_STATE_POW_FET_TEMP                0x00080000
#define PC_STATE_POW_WDG_TEMP                0x00100000
#define PC_STATE_POW_SHORT_CURR              0x00200000
#define PC_STATE_POW_HALLERR                 0x00400000
#define PC_STATE_POW_INTEGRAL_ERR            0x00800000

#define PC_STATE_CPU_OVERLOAD                0x01000000
#define PC_STATE_BEYOND_HARD                 0x02000000
#define PC_STATE_BEYOND_SOFT                 0x04000000
#define PC_STATE_POW_SETUP_ERR               0x08000000

/*
  R/W Digital IO (accessed with parameter ID 0x26)
*/
#define PC_DIOID_MOD_INBIT0                  0x00000001
#define PC_DIOID_MOD_INBIT1                  0x00000002
#define PC_DIOID_MOD_INBIT2                  0x00000004
#define PC_DIOID_MOD_INBIT3                  0x00000008
#define PC_DIOID_MOD_OUTBIT0                 0x00000010
#define PC_DIOID_MOD_OUTBIT1                 0x00000020
#define PC_DIOID_MOD_OUTBIT2                 0x00000040
#define PC_DIOID_MOD_OUTBIT3                 0x00000080
#define PC_DIOID_MOD_INSWR                   0x00000100
#define PC_DIOID_MOD_INSW1                   0x00000200
#define PC_DIOID_MOD_INSW2                   0x00000400


/*
  Module configuration (accessed with parameter ID 0x58)
*/
// Read only
//#define PC_SETUPID_MOD_ENCODER_FEEDBACK      0x00000001L // Not used
//#define PC_SETUPID_MOD_RESOLVER_FEEDBACK     0x00000002L // Not used
//#define PC_SETUPID_MOD_ABSOLUTE_FEEDBACK     0x00000004L // Not used
#define PC_SETUPID_MOD_4IN_4OUT              0x00000008L
#define PC_SETUPID_MOD_3IN_ENCODER_IN        0x00000010L
#define PC_SETUPID_MOD_3IN_ENCODER_OUT       0x00000020L
#define PC_SETUPID_MOD_RS232                 0x00000040L
#define PC_SETUPID_MOD_CAN                   0x00000200L
#define PC_SETUPID_MOD_PROFIBUS              0x00000400L
#define PC_SETUPID_MOD_USE_M3ID              0x00000800L
#define PC_SETUPID_MOD_USE_M4ID              0x00001000L

#define PC_SETUPID_MOD_USE_CANOPEN           0x00002000L
#define PC_SETUPID_MOD_USE_SW2_AS_ENABLE     0x00008000L
#define PC_SETUPID_MOD_USE_SW2_AS_BRAKE      0x00010000L
#define PC_SETUPID_MOD_ERROR_TO_OUT0         0x00020000L


/*
  Module configuration (accessed with parameter ID 0x39)
*/

// Writeable
#define PC_CONFIGID_MOD_BRAKE_PRESENT        0x00000008L
#define PC_CONFIGID_MOD_BRAKE_AT_POWERON     0x00000010L
#define PC_CONFIGID_MOD_SWR_WITH_ENCODERZERO 0x00000020L
#define PC_CONFIGID_MOD_SWR_AT_FALLING_EDGE  0x00000040L
#define PC_CONFIGID_MOD_CHANGE_SWR_TO_LIMIT  0x00000080L
#define PC_CONFIGID_MOD_SWR_ENABLED          0x00000100L
#define PC_CONFIGID_MOD_SWR_LOW_ACTIVE       0x00000200L
#define PC_CONFIGID_MOD_SWR_USE_EXTERNAL     0x00000400L

#define PC_CONFIGID_MOD_SW1_ENABLED          0x00000800L
#define PC_CONFIGID_MOD_SW1_LOW_ACTIVE       0x00001000L
#define PC_CONFIGID_MOD_SW1_USE_EXTERNAL     0x00002000L
#define PC_CONFIGID_MOD_SW2_ENABLED          0x00004000L
#define PC_CONFIGID_MOD_SW2_LOW_ACTIVE       0x00008000L
#define PC_CONFIGID_MOD_SW2_USE_EXTERNAL     0x00010000L
#define PC_CONFIGID_MOD_LINEAR               0x00020000L
#define PC_CONFIGID_MOD_ALLOW_FULL_CUR       0x00080000L

#define PC_CONFIGID_MOD_M3_COMPATIBLE        0x00100000L
#define PC_CONFIGID_MOD_LINEAR_SCREW         0x00200000L
#define PC_CONFIGID_MOD_DISABLE_ON_HALT      0x00800000L
#define PC_CONFIGID_MOD_WATCHDOG_ENABLE      0x01000000L

#define PC_CONFIGID_MOD_ZERO_MOVE_AFTER_HOK  0x02000000L
#define PC_CONFIGID_MOD_DIASBLE_ACK          0x04000000L
#define PC_CONFIGID_MOD_SYNC_MOTION          0x08000000L


/*
  Baudrates
*/
#define PC_BAUDRATE_250K                     1
#define PC_BAUDRATE_500K                     2
#define PC_BAUDRATE_1M                       3

/*
  Misc
*/
#define PC_WATCHDOG_TIMER_PERIOD_NS          40000000 // = 40 ms

#endif  //#ifndef _powercube_defines_H
