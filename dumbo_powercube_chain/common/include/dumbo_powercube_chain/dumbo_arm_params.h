#ifndef _dumbo_arm_params_h_

#define _dumbo_arm_params_h_

#include <dumbo_powercube_chain/utils.h>

// DH parameters follow the "Craig" convention...
// See: Craig, J.J. 
// "Introduction to Robotics: Mechanics and Control"

#define DUMBO_DH_ALPHA_0 (0)
#define DUMBO_DH_ALPHA_1 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_2 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_3 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_4 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_5 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_6 (-90*(PI/180.0))
#define DUMBO_DH_ALPHA_7 (0*(PI/180.0))

#define DUMBO_DH_A_0 (0)
#define DUMBO_DH_A_1 (0)
#define DUMBO_DH_A_2 (0)
#define DUMBO_DH_A_3 (0)
#define DUMBO_DH_A_4 (0)
#define DUMBO_DH_A_5 (0)
#define DUMBO_DH_A_6 (0)
#define DUMBO_DH_A_7 (0)

// needs to be measured in reality
#define DUMBO_DH_D_1 (0)
#define DUMBO_DH_D_2 (0)
#define DUMBO_DH_D_3 (0.313)
#define DUMBO_DH_D_4 (0)
#define DUMBO_DH_D_5 (0.2665)
#define DUMBO_DH_D_6 (0)
#define DUMBO_DH_D_7 (0)
#define DUMBO_DH_D_8 (0)

#define DUMBO_DH_THETA_1 (0)
#define DUMBO_DH_THETA_2 (180*(PI/180.0))
#define DUMBO_DH_THETA_3 (180*(PI/180.0))
#define DUMBO_DH_THETA_4 (180*(PI/180.0))
#define DUMBO_DH_THETA_5 (180*(PI/180.0))
#define DUMBO_DH_THETA_6 (180*(PI/180.0))
#define DUMBO_DH_THETA_7 (180*(PI/180.0))
#define DUMBO_DH_THETA_8 (0*(PI/180.0))

// needs to be measured in reality
#define DUMBO_TOOL_LENGTH (0.42)

// needs to be measured in reality
#define DUMBO_TOOL_LENGTH_L (0.42)//(0.29)
#define DUMBO_TOOL_LENGTH_R (0.42)

#define DUMBO_LINK1_LENGTH 0.274

#define  LOWER_CUBE_LIMIT_1 (-PI)
#define  LOWER_CUBE_LIMIT_2 (-125*(PI/180))
#define  LOWER_CUBE_LIMIT_3 (-PI)
#define  LOWER_CUBE_LIMIT_4 (-125*(PI/180))
#define  LOWER_CUBE_LIMIT_5 (-PI)
#define  LOWER_CUBE_LIMIT_6 (-125*(PI/180))
#define  LOWER_CUBE_LIMIT_7 (-PI)
#define  LOWER_CUBE_LIMIT_G (0)

#define  UPPER_CUBE_LIMIT_1 (PI)
#define  UPPER_CUBE_LIMIT_2 (125*(PI/180))
#define  UPPER_CUBE_LIMIT_3 (PI)
#define  UPPER_CUBE_LIMIT_4 (125*(PI/180))
#define  UPPER_CUBE_LIMIT_5 (PI)
#define  UPPER_CUBE_LIMIT_6 (125*(PI/180))
#define  UPPER_CUBE_LIMIT_7 (PI)
#define  UPPER_CUBE_LIMIT_G (0.06)


// end effector mass
#define DUMBO_EE_MASS 1.69

// FT sensor position relative to the wrist frame
#define DUMBO_FT_POS_L  0.218

// position of the end effector's center of mass
// relative to the wrist frame
#define DUMBO_CM_POS_L 0.078+0.218


#endif // #ifndef _dumbo_arm_params_h_
