/*
 * dumbo_arm_params
 *
 *  Created on: Aug 30, 2012
 *  Authors:   Christian Smith       Francisco Viña
 *              ccs <at> kth.se      fevb <at> kth.se
 */

/* Copyright (c) 2012, Christian Smith, Francisco Viña, CVAP, KTH
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
