#ifndef _dumbo_kinematics_h_

#define _dumbo_kinematics_h_

#define PRINT_JOINTS(T) fprintf(stdout, #T ":\n"); print_joint((T))
#define PRINT_POS(P) fprintf(stdout, #P ":\n"); print_pos((P))


struct joint_t{
  double j[7];
  double gripper;
};

struct pos_t{
  double x;
  double y;
  double z;

  double p;
  double t;
  double a;

  double phi;

  double gripper;
};

struct jac_t{
  double J[7][7];
};

struct vel_screw_t{
  double v[3]; // translational velocity
  double w[3]; // angular velocity
  double w_phi; // phi-angular velocity
};

// 'to' and 'from' can be between 
// 0 (the arm's 'zero' frame located at the second joint of each arm) 
// and 8 (tool frame).
struct mat4 generateT(int to, int from, struct joint_t theta);

// pos is expressed in the global-base frame
struct joint_t inv_kin_L(struct pos_t pos, double t5);
struct joint_t inv_kin_R(struct pos_t pos, double t5);

// left for backwards compatibility, do not use
// pos is expressed in the arm's zero frame.
struct joint_t inv_kin(struct pos_t pos, double t5);


// returns the pose of 'n_frame'
// with respect to the global reference frame
// This base frame is located between
// both arms, on the axis of the first joints.
// 
// Z axis is vertical and points up.
// 
// X axis is horizontal and points towards the 
// right arm (serial number of first joint == 41253).
// 
// Y axis points towards the front of the robot.
//
// Frame 8 is the tool frame, on the tip of the tool.
struct pos_t fwd_kin_L(struct joint_t theta, int n_frame);
struct pos_t fwd_kin_R(struct joint_t theta, int n_frame);


// Calculates the jacobian matrix of the arm
// given a configuration in joint space. The jacobian is calculated
// on the tip of the end effector with respect to the base frame.
struct jac_t jacob_L(struct joint_t theta);
struct jac_t jacob_R(struct joint_t theta);


// Calculates a velocity screw given a jacobian matrix
// and a joint velocity vector
struct vel_screw_t fwd_vel(struct jac_t jacobian, struct joint_t vel);

// Calculates the inverse jacobian matrix. 
struct jac_t inv_jacob(struct jac_t jacobian);

// Calculates the joint velocities required to generate
// the desired vel screw
struct joint_t inv_vel(struct jac_t inv_jacobian, struct vel_screw_t vel_screw);

// transforms a pos 
// to a homogenic transformation matrix
struct mat4 pos_2_T(struct pos_t pos);

// transforms a homogenic transformation matrix
// to a pos_t structure (x,y,z and p,t,a (ZYZ Euler angles))
struct pos_t T_2_pos(struct mat4 T);

// returns ZYZ-euler angles that represent
// the rotation matrix contained in
// the transformation matrix T. The angles
// are placed in the pos structure.
struct pos_t T_2_euler(struct mat4 T);


// returns the rotation matrix corresponding
// to a homogeneous transformation T
struct mat4 T_2_rot(struct mat4 T);

// returns the phi parameter corresponding
// to the joint configuration 'theta'.
double fwd_kin_phi(struct joint_t theta);

// sets to zero a given joint velocity
void vel_set_zero(struct joint_t *joint_vel);

void print_pos(struct pos_t pos);
void print_joint(struct joint_t theta);
void print_joint_vel(struct joint_t vel);
void print_jacob(struct jac_t jacobian);
void print_vel_screw(struct vel_screw_t vel_screw);

#endif 
