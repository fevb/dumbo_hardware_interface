
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
