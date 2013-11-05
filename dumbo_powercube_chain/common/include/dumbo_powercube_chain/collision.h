/*
 *  collision
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

#ifndef _collision_h_

#define _collision_h_

#include <dumbo_powercube_chain/dumbo_kinematics.h>

// JL: Joint Limit
// CL: Cartesian Limit
//

// Returns -1 when a joint limit has been reached,
// 0 otherwise.
int col_JL_detect_L(struct joint_t theta);
int col_JL_detect_R(struct joint_t theta);


// Returns -1 when the end effector is positioned 
// outside a box defined in cartesian space, returns 0
// otherwise.
// The purpose of this box is to keep the end effector 
// within some cartesian position limits 
// in order to avoid collisions with the platform
// or the torso
int col_CL_detect_L(struct pos_t pos);
int col_CL_detect_R(struct pos_t pos);


// Returns 0 if the given joint configuration 
// does not generate collisions (both joint and cartesian)
// Returns -1 otherwise.
int col_detect_L(struct joint_t theta);
int col_detect_R(struct joint_t theta);


// Used in velocity control mode.
// Saturates the joint velocities to avoid reaching
// the joint limits. Returns the saturated velocity vector.
// Requires as input the desired joint velocity as well as 
// the current joint angles (thea) and the target acceleration.
struct joint_t col_sat_vel_L(struct joint_t vel, struct joint_t theta,
			     struct joint_t target_acc, double vel_control_freq);
struct joint_t col_sat_vel_R(struct joint_t vel, struct joint_t theta, 
			     struct joint_t target_acc, double vel_control_freq);


// Used in velocity control mode.
// When the end effector is outside the
// cartesian box and the arm is in velocity control mode, 
// we want the end effector to move towards the inside of the
// cartesian box with a proper velocity command.
// 
// Returns 0 if the input vel vector
// moves the end-effector towards the inside 
// of the safety cartesian box, returns -1 otherwise.
int col_check_vel_L(struct joint_t vel, struct joint_t theta);
int col_check_vel_R(struct joint_t vel, struct joint_t theta);

#endif
