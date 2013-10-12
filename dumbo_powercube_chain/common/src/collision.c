#include <dumbo_powercube_chain/collision.h>
#include <dumbo_powercube_chain/collision_defines.h>
#include <dumbo_powercube_chain/utils.h>
#include <dumbo_powercube_chain/dumbo_kinematics.h>
#include <math.h>
#include <stdio.h>

static double j_min_L[8] = {MIN_J1_L,
			    MIN_J2_L,
			    MIN_J3_L,
			    MIN_J4_L,
			    MIN_J5_L,
			    MIN_J6_L,
			    MIN_J7_L,
			    MIN_G_L};

static double j_max_L[8] = {MAX_J1_L,
			    MAX_J2_L,
			    MAX_J3_L,
			    MAX_J4_L,
			    MAX_J5_L,
			    MAX_J6_L,
			    MAX_J7_L,
			    MAX_G_L};


static double j_min_R[8] = {MIN_J1_R, 
			    MIN_J2_R, 
			    MIN_J3_R, 
			    MIN_J4_R, 
			    MIN_J5_R, 
			    MIN_J6_R, 
			    MIN_J7_R, 
			    MIN_G_R};

static double j_max_R[8] = {MAX_J1_R,
			    MAX_J2_R,
			    MAX_J3_R,
			    MAX_J4_R,
			    MAX_J5_R,
			    MAX_J6_R,
			    MAX_J7_R,
			    MAX_G_R};


static double cart_box_L[6] = {MIN_X_L,
			       MIN_Y_L,
			       MIN_Z_L,
			       MAX_X_L,
			       MAX_Y_L,
			       MAX_Z_L};
				     

static double cart_box_R[6] = {MIN_X_R,
			       MIN_Y_R,
			       MIN_Z_R,
			       MAX_X_R,
			       MAX_Y_R,
			       MAX_Z_R};


double _sat_vel(double vel, double theta, double target_acc, 
		double jlim, double vel_control_freq);

int col_JL_detect_L(struct joint_t theta){

  // checking joint 1
  if((theta.j[0]<j_min_L[0]) || (theta.j[0]>j_max_L[0])) return -1;

  // checking joint 2
  if((theta.j[1]<j_min_L[1]) || (theta.j[1]>j_max_L[1])) return -1;

  // checking joint 3
  if((theta.j[2]<j_min_L[2]) || (theta.j[2]>j_max_L[2])) return -1;

  // checking joint 4
  if((theta.j[3]<j_min_L[3]) || (theta.j[3]>j_max_L[3])) return -1;

  // checking joint 5
  if((theta.j[4]<j_min_L[4]) || (theta.j[4]>j_max_L[4])) return -1;

  // checking joint 6
  if((theta.j[5]<j_min_L[5]) || (theta.j[5]>j_max_L[5])) return -1;

  // checking joint 7
  if((theta.j[6]<j_min_L[6]) || (theta.j[6]>j_max_L[6])) return -1;

  // checking gripper
  if((theta.gripper<j_min_L[7]) || (theta.gripper>j_max_L[7])) return -1;
  
  return 0;

}


int col_JL_detect_R(struct joint_t theta){

  // checking joint 1
  if((theta.j[0]<j_min_R[0]) || (theta.j[0]>j_max_R[0])) return -1;

  // checking joint 2
  if((theta.j[1]<j_min_R[1]) || (theta.j[1]>j_max_R[1])) return -1;

  // checking joint 3
  if((theta.j[2]<j_min_R[2]) || (theta.j[2]>j_max_R[2])) return -1;

  // checking joint 4
  if((theta.j[3]<j_min_R[3]) || (theta.j[3]>j_max_R[3])) return -1;

  // checking joint 5
  if((theta.j[4]<j_min_R[4]) || (theta.j[4]>j_max_R[4])) return -1;

  // checking joint 6
  if((theta.j[5]<j_min_R[5]) || (theta.j[5]>j_max_R[5])) return -1;

  // checking joint 7
  if((theta.j[6]<j_min_R[6]) || (theta.j[6]>j_max_R[6])) return -1;

  // checking gripper
  if((theta.gripper<j_min_R[7]) || (theta.gripper>j_max_R[7])) return -1;
  
  return 0;

}


int col_CL_detect_L(struct pos_t pos){

  if((pos.x<cart_box_L[min_x]) || (pos.x>cart_box_L[max_x])) return -1;

  if((pos.y<cart_box_L[min_y]) || (pos.y>cart_box_L[max_y])) return -1;

  if((pos.z<cart_box_L[min_z]) || (pos.z>cart_box_L[max_z])) return -1;

  return 0;

}


int col_CL_detect_R(struct pos_t pos){

  if((pos.x<cart_box_R[min_x]) || (pos.x>cart_box_R[max_x])) return -1;

  if((pos.y<cart_box_R[min_y]) || (pos.y>cart_box_R[max_y])) return -1;

  if((pos.z<cart_box_R[min_z]) || (pos.z>cart_box_R[max_z])) return -1;

  return 0;

}


int col_detect_L(struct joint_t theta){

  int ret = 0;
  struct pos_t pos;

  pos = fwd_kin_L(theta, 8);

  ret += col_JL_detect_L(theta);
  ret += col_CL_detect_L(pos);

  if(ret<0) ret = -1;

  return ret;
}

int col_detect_R(struct joint_t theta){

  int ret = 0;
  struct pos_t pos;
  
  pos = fwd_kin_R(theta, 8);

  ret += col_JL_detect_R(theta);
  ret += col_CL_detect_R(pos);

  if(ret<0) ret = -1;

  return ret;
}


struct joint_t col_sat_vel_L(struct joint_t vel, struct joint_t theta,
			     struct joint_t target_acc, double vel_control_freq){

  struct joint_t sat_j_vel;
  
  int i;
  double jlim;

  
  // loop through joint angles
  for(i=0;i<7;i++){
    
    // if velocity is zero, no need to saturate
    if(vel.j[i]==0){
      sat_j_vel.j[i] = 0;
    }

    else{
      if(vel.j[i]>0){
	jlim = j_max_L[i];
      }

      else{
	jlim = j_min_L[i];
      }

      sat_j_vel.j[i] = _sat_vel(vel.j[i], theta.j[i], target_acc.j[i], 
				jlim, vel_control_freq);
    }
  }


  return sat_j_vel;
}



struct joint_t col_sat_vel_R(struct joint_t vel, struct joint_t theta,
			     struct joint_t target_acc, double vel_control_freq){

  struct joint_t sat_j_vel;
  
  int i;
  double jlim;

  
  // loop through joint angles
  for(i=0;i<7;i++){
    
    // if velocity is zero, no need to saturate
    if(vel.j[i]==0){
      sat_j_vel.j[i] = 0;
    }

    else{
      if(vel.j[i]>0){
	jlim = j_max_R[i];
      }

      else{
	jlim = j_min_R[i];
      }

      sat_j_vel.j[i] = _sat_vel(vel.j[i], theta.j[i], target_acc.j[i], 
				jlim, vel_control_freq);
    }
  }

  return sat_j_vel;
}


int col_check_vel_L(struct joint_t vel, struct joint_t theta){

  int ret;
  struct pos_t pos;
  struct jac_t jacobian;
  struct vel_screw_t vel_screw;
  double vx, vy, vz;

  ret = 0;

  // calculate position
  pos = fwd_kin_L(theta, 8);

  // calculate jacobian + vel screw
  jacobian = jacob_L(theta);
  vel_screw = fwd_vel(jacobian, vel);

  vx = vel_screw.v[0];
  vy = vel_screw.v[1];
  vz = vel_screw.v[2];

  // make sure the translational velocity makes 
  // the end-effector move towards
  // the inside of the safety-cartesian box
  if((pos.x<cart_box_L[min_x])){
    if(vx<0) ret-=1;
  }

  if((pos.x>cart_box_L[max_x])){
    if(vx>0) ret-=1;
  }

  if((pos.y<cart_box_L[min_y])){
    if(vy<0) ret-=1;
  }

  if((pos.y>cart_box_L[max_y])){
    if(vy>0) ret-=1;
  }

  if((pos.z<cart_box_L[min_z])){
    if(vz<0) ret-=1;
  }

  if((pos.z>cart_box_L[max_z])){
    if(vz>0) ret-=1;
  }

  if(ret<0) ret = -1;

  return ret;
}


int col_check_vel_R(struct joint_t vel, struct joint_t theta){

  int ret;
  struct pos_t pos;
  struct jac_t jacobian;
  struct vel_screw_t vel_screw;
  double vx, vy, vz;

  ret = 0;

  // calculate position
  pos = fwd_kin_R(theta, 8);

  // calculate jacobian + vel screw
  jacobian = jacob_R(theta);
  vel_screw = fwd_vel(jacobian, vel);

  vx = vel_screw.v[0];
  vy = vel_screw.v[1];
  vz = vel_screw.v[2];

  // make sure the translational velocity makes 
  // the end-effector move towards
  // the inside of the safety-cartesian box
  if((pos.x<cart_box_R[min_x])){
    if(vx<0) ret-=1;
  }

  if((pos.x>cart_box_R[max_x])){
    if(vx>0) ret-=1;
  }

  if((pos.y<cart_box_R[min_y])){
    if(vy<0) ret-=1;
  }

  if((pos.y>cart_box_R[max_y])){
    if(vy>0) ret-=1;
  }

  if((pos.z<cart_box_R[min_z])){
    if(vz<0) ret-=1;
  }

  if((pos.z>cart_box_R[max_z])){
    if(vz>0) ret-=1;
  }
  
  if(ret<0) ret = -1;

  return ret;
}


double _sat_vel(double vel, double theta, double target_acc, 
		double jlim, double vel_control_freq){

  double sat_vel, max_vel;
  double inv_acc, inv_freq;
  double _4ac, _2a, _b;

  sat_vel = vel;
  
  inv_freq = 1/vel_control_freq;
  inv_acc = 1/target_acc;
    
  // if velocity is zero, no need to saturate
  if(vel==0){
    sat_vel = 0;
    return sat_vel;
  }

  else{

    _4ac = 2*inv_acc*(theta-jlim);
    _2a = inv_acc;
    _b = inv_freq;

    // calculate maximum vel
    if(vel>0){
      if(_4ac<0){
	max_vel = ((-1*_b)+sqrt(_b*_b - _4ac))/_2a;
      }

      // in this case the joint angle has
      // surpassed the limit
      else{
	max_vel = 0;
      }
    }

    else{
      if(_4ac>0){
	max_vel = -1*((-1*_b)+sqrt(_b*_b + _4ac))/_2a;
      }
	
      // in this case the joint angle has
      // surpassed the limit
      else{
	max_vel = 0;
      }
    }
    
    // if desired vel is greater than the 
    // maximum allowed vel, then saturate
    if(fabs(vel) > fabs(max_vel)){
      sat_vel = max_vel;
    }

    else{
      sat_vel = vel;
    }
  }   

  return sat_vel;

}
