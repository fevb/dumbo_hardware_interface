// Code by Christian Smith, Dec 2009

// Please remember to disconnect the charger before you turn on power
// to the arm. The arm will get 27V otherwise hardware might break.


#ifndef _POWERARM_H_

#define _POWERARM_H_

#include <dumbo_powercube_chain/dumbo_kinematics.h>
/* #include "dumbo_dynamics.h" */
#include <pthread.h>

// Some data types used to send commands:
struct velacc_struct{
  float vel[7];
  float acc[7];
};

// defines arm selection (LEFT or RIGHT ARM)  data type
#ifndef ARMSELECT
enum arm {LEFT_ARM, RIGHT_ARM};
typedef enum arm arm_select;
#define ARMSELECT 1
#endif

// All functions for the powerarm have "pa_" prefix


// Initializes the Kvaser CAN card,
// finds the channel with the arm on it,
// and get the arm's status
// returns 0 on success, -1 if no arm was found or CAN error
// returns -2 if arm reported any errors.
// Call this function first
// This is a blocking call.
//
// LR selects the LEFT or RIGHT arm.
// (see LEFT_ARM RIGHT_ARM)
/* int pa_connect(arm_select LR); */
int pa_connect(arm_select LR);


// Disconnects the CAN card.
// Should be called last.
// Returns 0 on success, -1 on CAN error
int pa_disconnect(void);

// Starts the watchdogs either
// on the left or right arm LR = LEFT_ARM or RIGHT_ARM
pthread_t pa_start_watchdog(arm_select LR);


// Homes the arm.
// Returns 0 on success, -1 on error.
// Call this function second after powering up
// the arm. If a module is already homed, 
// the command is ignored.
// This is a blocking call.
int pa_home(arm_select LR);

// Resets the modules, sets the hardware limits
// and sets low velocity limits.
// Returns 0 on success, -1 on error.
// Call this function third.
// This is a blocking call.
int pa_reset(arm_select LR);


// Returns -1 if there is an error
// in any module of the arm.
// Returns 0 otherwise.
int pa_detect_error(arm_select LR);

// Returns -1 if there is an error
// in any module of the arm.
// Returns 0 otherwise.
int pa_get_state(int moduleID, unsigned long int *status, arm_select LR);

// Tells the arm to move to a specific
// cartesian position.
// This does not give coordinated motion
// Returns 0, on success, -1 on cube error
// EINVAL on illegal position
// Arm will not recieve new
// motion comand if pos is illegal.
// The call blocks until all comands have beens
// sent to the cubes, but not until motion
// is completed.
// New command can be sent before old is
// completed, the arm will the move towards
// the new goal instead.
//int pa_goto_cart(struct dumbo_position_struct pos);

// Tells the arm to move to a specific
// cartesian position.
// This gives coordinated motion along a
// linear path in cartesian space,
// and takes t seconds to complete.
// The user is responsible for setting 
// reasonable values for t - bad values
// will give uglier paths. Reasonable 
// values for t depend on current velocity
// settings and inv. kinematics jacobian.
// Returns 0, on success, -1 on cube error
// EINVAL on illegal goal position
// The call blocks until motion
// is completed.
//int pa_goto_cart_linear(struct dumbo_position_struct pos, double t);

// Tells the arm to move to a specific
// configuration.
// This does not give coordinated motion
// Returns 0, on success, -1 on cube error
// EINVAL on illegal position
// The call blocks until all comands have beens
// sent to the cubes, but not until motion
// is completed.
// New command can be sent before old is
// completed, the arm will the move towards
// the new goal instead.
//
// Collision detection is made inside these functions
// 
// NB: 'pos' argument is considered to be expressed 
// in the global base frame...
// see dumbo_kinematics.c for more details.
// This base frame is located between
// both arms, on the axis of the first joints, .
// 
// Z axis is vertical and points up.
// 
// X axis is horizontal and points towards the 
// right arm (serial number of first joint == 41253).
// 
// Y axis points towards the front of the robot.
//
int pa_goto_angle(struct joint_t theta, arm_select LR);

int pa_goto_pos(struct pos_t pos, arm_select LR);

// Sends velocity commands to joints
// saturates velocities in order to avoid 
// reaching the joint limits.
// No collision detection is made inside this function.
int pa_goto_vel(struct joint_t vel, arm_select LR);

// Returns 0 when the arm has reached its
// final position after a goto_angle or
// goto_pos command has been issued, 
// returns -1 otherwise. 
// The call blocks until cubes have answered
int pa_ramp_ended(arm_select LR);

// Gets the current angle position of the joints.
// returns 0 on success, -1 on cube error.
// Call blocks until cubes have answered 
// (or timed out).
int pa_get_angle(struct joint_t *theta,
		 arm_select LR);

// Gets the last angle position of the arm,
// should be used after a motion command in order to use 
// the acknowledge and achieve a higher control frequency
int pa_get_last_angle(struct joint_t *theta, arm_select LR);

// Gets the current velocity of the joints
// returns 0 on success, -1 on cube error.
// Call blocks until cubes have answered
// (or timed out). 
int pa_get_vel(struct joint_t *vel,
		arm_select LR);


// Returns 0 success.
// Returns -1 on cube error. 
int pa_get_target_vel(struct joint_t *target_vel, arm_select LR);

// Returns 0 success.
// Returns -1 on cube error. 
int pa_set_target_vel(struct joint_t target_vel, arm_select LR);



// Returns 0 success.
// Returns -1 on cube error. 
int pa_get_target_acc(struct joint_t *target_acc, arm_select LR);

// Returns 0 success.
// Returns -1 on cube error. 
int pa_set_target_acc(struct joint_t target_acc, arm_select LR);


//int pa_get_pos_for_response(struct position_response_packet *utdata);


//int pa_set_max_acc(struct velacc_struct velacc);
//int pa_set_max_vel(struct velacc_struct velacc);

// Sets the velocities and acceleration
// limits to user-defined values.
// reurns 0 on success,
// -1 on cube error, EINVAL on invalid values 
// This is a blocking call
//int pa_set_velacc_lim(struct velacc_struct velacc);

// Gets the current set values for velocities 
// and acceleration limits.
// reurns 0 on success,
// -1 on cube error.
// This is a blocking call
//int pa_get_velacc_lim(struct velacc_struct *velacc);

// Gets the inverse kinematic solution to a 
// cartesian position.
// Returns 0 on success, EINVAL on unreachable
// point in indata.
//int pa_inv_kin(struct dumbo_position_struct pos,
//	       struct dumbo_angle_struct *theta);

// Gets the forward kinematic solution to a 
// configuration in joint space.
// The pose is expressed in the global base frame.
struct pos_t pa_forw_kin(struct joint_t theta, arm_select LR);
	       


// sets the gripper position
// unit should be centimeters (?)
// returns 0 on success, -1 on failure,
// and EINVAL on illegal indata.
// blocks until communications is finished,
// does not await motion results.
// If new command is sent before
// goal is reached, new command will take precedence.
//int pa_set_gripper(float pos);


// gets the gripper position
// unit should be meters (?)
// returns 0 on success, -1 on failure.
// blocks until communications is finished,
// does not await motion results.
// If new command is sent before
// goal is reached, new command will take precedence.
//int pa_get_gripper(float *pos);


// Imediately halts arm.
// may damage arm if it is moving fast.
// To resume action after a halt, you
// will need to reset.
// returns 0 on success, -1
// on error. 
// HALTING IS NOT RELIABLE ENOUGH FOR SAFETY
// ALWAYS HAVE THE EMERGENCY STOP READY!!!!
//
// LR selects either the LEFT or RIGHT arm
int pa_halt(arm_select LR);

// Sends a zero joint velocity command to the arm
// return 0 on success, -1 on error
int pa_stop(arm_select LR);

// parking procedures
// this call blocks until the arm is parked. *** have to be implemented
int pa_park(arm_select LR);
int pa_park_L(void);
int pa_park_R(void);

// multiplies joint angles 1,3,4,5,6,7 
// by -1 to transform them to the schunk modules'
// internal coordinate system.
struct joint_t pa_correct_theta(struct joint_t theta);

// returns the FT sensor readings.
/* struct FT_t pa_get_FT(void); */

// collision detection functions (see collision.h)

// Returns 0 if the given theta angles produce a collision
// Returns -1 otherwise.
int pa_col_detect(struct joint_t theta, arm_select LR);


// Saturates a joint velocity vector in order to avoid 
// reaching the joint limits
struct joint_t pa_col_sat_vel(struct joint_t vel, struct joint_t theta,
			      struct joint_t target_acc, double vel_control_freq,
			      arm_select LR);

// Checks if a given joint velocity vector keeps
// the end-effector within a safety- cartesian box
// Returns 0 if the joint vel is OK.
// Returns -1 otherwise.
int pa_col_check_vel(struct joint_t vel, struct joint_t theta, arm_select LR);


// sends velocity commands to the 2nd and 5th joint
int pa_goto_vel_DO(struct joint_t vel, arm_select LR);

// kills arm watchdogs
void pa_kill_watchdogs(pthread_t watchdog_t);

#endif

