#include <dumbo_powercube_chain/dumbo_kinematics.h>
#include <dumbo_powercube_chain/dumbo_arm_params.h>
#include <dumbo_powercube_chain/homogenic_vectors.h>
#include <math.h>
#include <dumbo_powercube_chain/utils.h>
#include <stdio.h>
#include <errno.h>

// The Denavit-Hartenberg parameters:
/* this should be moved to dumbo_arm_params.h
 */

struct vec4 tool = {.v = {0, 0, DUMBO_TOOL_LENGTH, 1}};
struct vec4 tool_L = {.v = {0, 0, DUMBO_TOOL_LENGTH_L, 1}};
struct vec4 tool_R = {.v = {0, 0, DUMBO_TOOL_LENGTH_R, 1}};

static double DH_al[8] = {DUMBO_DH_ALPHA_0,
			 DUMBO_DH_ALPHA_1,
			 DUMBO_DH_ALPHA_2,
			 DUMBO_DH_ALPHA_3,
			 DUMBO_DH_ALPHA_4,
			 DUMBO_DH_ALPHA_5,
			 DUMBO_DH_ALPHA_6,
			 DUMBO_DH_ALPHA_7};

static double DH_a[8] = {DUMBO_DH_A_0,
			 DUMBO_DH_A_1,
			 DUMBO_DH_A_2,
			 DUMBO_DH_A_3,
			 DUMBO_DH_A_4,
			 DUMBO_DH_A_5,
			 DUMBO_DH_A_6,
			 DUMBO_DH_A_7};

static double DH_d[8] = {DUMBO_DH_D_1,
			 DUMBO_DH_D_2,
			 DUMBO_DH_D_3,
			 DUMBO_DH_D_4,
			 DUMBO_DH_D_5,
			 DUMBO_DH_D_6,
			 DUMBO_DH_D_7,
			 DUMBO_DH_D_8};

static double DH_th[8] = {DUMBO_DH_THETA_1,
			  DUMBO_DH_THETA_2,
			  DUMBO_DH_THETA_3,
			  DUMBO_DH_THETA_4,
			  DUMBO_DH_THETA_5,
			  DUMBO_DH_THETA_6,
			  DUMBO_DH_THETA_7,
			  DUMBO_DH_THETA_8};


// Transformation matrices for converting 
// from the arms' (zero-) frame 
// to a global base frame that we define as follows:
//
// This global base frame is located between
// the first joint of the left arm
// and the first joint of the right arm.
// 
// Z axis is vertical and points up.
// 
// X axis is horizontal and points towards the 
// right arm (serial number of first joint == 41253).
// 
// Y axis points towards the front of the robot.
struct mat4 T_arm2base_L = {.m = {{  0,  0, -1, -DUMBO_LINK1_LENGTH },
				  { -1,  0,  0,    0   },
				  {  0,  1,  0,    0   },
				  {  0,  0,  0,    1   }}};

struct mat4 T_arm2base_R = {.m = {{  0,  0,  1,  DUMBO_LINK1_LENGTH },
				  {  1,  0,  0,    0   },
				  {  0,  1,  0,    0   },
				  {  0,  0,  0,    1   }}};


// Position of the FT sensor relative to the wrist frame
struct mat4 T_w_ft_L = {.m = {{  1,  0,  0,    0   },
			      {  0,  1,  0,    0   },
			      {  0,  0,  1, DUMBO_FT_POS_L },
			      {  0,  0,  0,    1   }}};

// location of the center of mass relative to the wrist frame
struct mat4 T_w_cm_L = {.m = {{  1,  0,  0,    0   },
			      {  0,  1,  0,    0   },
			      {  0,  0,  1, DUMBO_CM_POS_L },
			      {  0,  0,  0,    1   }}};



struct mat4 generateT(int to, int from, struct joint_t theta){
  struct mat4 T;
  int k;
  double _theta[8] = {0,0,0,0,0,0,0,0};
  double c,s,ca,sa,a,d;

  // If both frames are the same, generate identity transform
  T = EYE4();

  // if the frames are different, calculate transform
  if(to != from){
    
    for(k=0;k<7;k++){
      _theta[k] = DH_th[k] + theta.j[k];
    }
    _theta[7] = DH_th[7];

    // generate transform matrices for all relevant steps:
    // (transform matrix generated according to p.84 in "Introduction to
    // robotics" by J.J. Craig) 
    for(k=MIN(to,from); k<(MAX(to,from)); k++){

     
      c = cos(_theta[k]);
      s = sin(_theta[k]);
      
      ca = cos(DH_al[k]);
      sa = sin(DH_al[k]);

      a = DH_a[k];
      d = DH_d[k];
      
      
      {
	struct mat4 Ti = {.m={{  c,       -s,     0,      a   },
			      { s*ca,    c*ca,   -sa,   -d*sa },
			      { s*sa,    c*sa,    ca,    d*ca },
			      {  0,        0,     0,      1   }}};

	//PRINT_MAT(Ti);
	T = mat_mat_mul(T,Ti);
      }
    }
  }
  

  // in the more common case, a coordinate in a higher number frame
  // is represented in a lower number frame. This has already been
  // calculated. In the reverse case, the inverse is needed:
  if (to>from){
    T=inv_mat(T);
  }

  return T;
}


struct pos_t fwd_kin_L(struct joint_t theta, int n_frame){
  struct pos_t pos = { .x = 0.0,
			 .y = 0.0,
			 .z = 0.0,
			 .p = 0.0,
			 .t = 0.0,
			 .a = 0.0,
			 
			 .phi = 0.0,
			 .gripper = theta.gripper};

  struct vec4 p, tool_temp; 
  struct mat4 T, T_BaseF;
  struct pos_t pos_temp;
  double phi;

  if((n_frame<1) || (n_frame>8)){
    printf("Error fwd kin\n");
    return pos;
  }

  T = generateT(0, n_frame, theta);
  
  // Transform to global base frame...
  T_BaseF = mat_mat_mul(T_arm2base_L,T);

  if(n_frame==8){
    tool_temp = tool;
    tool = tool_L;
    p = mat_vec_mul(T_BaseF,tool);
    tool = tool_temp;

    pos.x = p.v[0];
    pos.y = p.v[1];
    pos.z = p.v[2];
  }

  else{
    pos.x = T_BaseF.m[0][3];
    pos.y = T_BaseF.m[1][3];
    pos.z = T_BaseF.m[2][3];
  }

  // extract ZYZ euler angles
  pos_temp = T_2_euler(T_BaseF);

  pos.p = pos_temp.p;
  pos.t = pos_temp.t;
  pos.a = pos_temp.a;  



  // calculate phi parameter
  phi = fwd_kin_phi(theta);

  pos.phi = phi;
  pos.gripper = theta.gripper;
    
  return pos;
}

struct pos_t fwd_kin_R(struct joint_t theta, int n_frame){
  struct pos_t pos = { .x = 0.0,
			 .y = 0.0,
			 .z = 0.0,
			 .p = 0.0,
			 .t = 0.0,
			 .a = 0.0,
			 
			 .phi = 0.0,
			 .gripper = theta.gripper};

  struct vec4 p, tool_temp; 
  struct mat4 T, T_BaseF;
  struct pos_t pos_temp;
  double phi;

  if((n_frame<1) || (n_frame>8)){
    printf("Error fwd kin\n");
    return pos;
  }

  T = generateT(0, n_frame, theta);
  
  // Transform to global base frame...
  T_BaseF = mat_mat_mul(T_arm2base_R,T);

  if(n_frame==8){
    tool_temp = tool;
    tool = tool_R;
    p = mat_vec_mul(T_BaseF,tool);
    tool = tool_temp;

    pos.x = p.v[0];
    pos.y = p.v[1];
    pos.z = p.v[2];
  }

  else{
    pos.x = T_BaseF.m[0][3];
    pos.y = T_BaseF.m[1][3];
    pos.z = T_BaseF.m[2][3];
  }

  // extract ZYZ euler angles
  pos_temp = T_2_euler(T_BaseF);

  pos.p = pos_temp.p;
  pos.t = pos_temp.t;
  pos.a = pos_temp.a;  



  // calculate phi parameter
  phi = fwd_kin_phi(theta);

  pos.phi = phi;
  pos.gripper = theta.gripper;
    
  return pos;
}


struct joint_t inv_kin(struct pos_t pos, double t5){
 

  // initilaize output struct.
  // gripper is just copied
  struct joint_t theta = { .j = {0,0,0,0,0,0,0},
			   .gripper = pos.gripper};

  // some variables to hold partial computations
  struct joint_t theta_m;

  double p,t,a,sp,st,sa,cp,ct,ca;

  double r11,r12,r13,r21,r22,r23,r31,r32,r33;

  double wr2, d32, d52;
  double alpha;

  // tool positions
  struct mat4 T,Tm;
  struct vec4 Target,TargetP,TargetM,tool_v;
  double ptx, pty, ptz, prel_a;

  // wrist positions
  double  wx, wy, wz;
  struct vec4 wv,link4,tool_proj,link3_proj,link3_perp;
  struct mat4 Rot;  // rotation of last frame
  double wr, wt,wp, tool_par, tool_perp;

  // elbow positions
  double eh, ehr, scale_er;
  struct vec4 e_rw,v_erw_e,ev,q,q_el,q_el_norm,perp_up_el,
    perp_left_el;

  double cwp, swp, cwt, swt, cphi, sphi;
  struct mat4 Rpan,Rtilt,Rphi; 

  double  angle_should_elb, dq,up_ang,q_up,q_left;



  // calculate rotation matrix for tool frame, using p,t,a
  p = pos.p;
  t = pos.t;
  a = pos.a;

  // precalculate values often needed:
  sp=sin(p);
  st=sin(t);
  sa=sin(a);
  cp=cos(p);
  ct=cos(t);
  ca=cos(a);

  //create rotation matrix:
  r13=cp*st;
  r23=sp*st;
  r33=ct;
  r12=-cp*ct*sa - sp*ca;
  r22=cp*ca - sp*ct*sa;
  r32=st*sa;

  // calculate the last column from the cross product
  r11 = r22*r33 - r23*r32;
  r21 = r32*r13 - r12*r33;
  r31 = r12*r23 - r13*r22;

  // Generate rotational matrix
  Rot = create_matrix(r11, r12, r13, 0,
		      r21, r22, r23, 0,
		      r31, r32, r33, 0,
		        0,   0,   0, 1);

  // Get position of end effector (tool)
  ptx = pos.x;
  pty = pos.y;
  ptz = pos.z;

  
  // Calculate position of wrist = pt-Rot*tool 
  wv = vec_vec_sub(create_vector(ptx,pty,ptz),
		   mat_vec_mul(Rot,tool));
  wx = wv.v[0];
  wy = wv.v[1];
  wz = wv.v[2];

  //PRINT_VEC(wv);


  // calculate wrist position in polar coordinates,
  // defining the pan angle as zero if we are looking
  // straight up. This is something that one might
  // want to change in real implementation, for instance
  // by defining pan as the last angle found for
  // non-zero tilt.
  wr = vec_norm(wv); // radius
  wt = (PI/2)-atan2(wz,sqrt((wx*wx) + (wy*wy)));   //tilt

  if(fabs(wt)<0.001){
    wp = 0;              //pan
  }else{
    wp = atan2(wy,wx);              //pan
  }

  // Reject unreachable position
  if(wr>(DH_d[2]+DH_d[4])){
    
    fprintf(stderr,"Unreachable position!\n");
    PRINT_VEC(wv);
    errno = -1;
    return theta;
  }

  // some reusable numbers:
  wr2 = wr*wr;
  d32 = DH_d[2]*DH_d[2];
  d52 = DH_d[4]*DH_d[4];

  // calculate elbow angle, th4
  // using law of cosines
  // a^2 = b^2 + c^2 - 2bc*cos(alpha)
  alpha = acos((d32+d52-wr2)/(2*DH_d[2]*DH_d[4]));
  theta.j[3] = PI-alpha;

  // calculate position of elbow. We find
  // the hight of the triangle and move this
  // distance from the line between shoulder
  // and wrist, using angle phi
  // Height is found using herons formula;
  // h_a = (2/a)*sqrt(p(p-a)(p-b)(p-c))
  p = (wr+DH_d[2]+DH_d[4])/2;
  eh = (2/wr)*sqrt(p*(p-wr)*(p-DH_d[2])*(p-DH_d[4]));

  // we find the length from shoulder to elbow, when
  // projected to the line between shoulder and wrist,
  // using Pythagora's formula

  ehr = sqrt(d32-(eh*eh));

  // we find e_rw, the position where he intersects rw
  scale_er = (ehr/wr);   
  e_rw = scal_vec_mul(scale_er,wv);
  
  // we then find v_erw_e, the vector from e_rw to the elbow
  // we use rotational matrices
  cwp = cos(wp);
  swp = sin(wp);
  cwt = cos(wt-(PI/2)); // due to definition of tilt=0 being straight up
  swt = sin(wt-(PI/2));
  cphi = cos(pos.phi);
  sphi = sin(pos.phi);
  
  Rpan = create_matrix(cwp, -swp, 0, 0,
		       swp,  cwp, 0, 0,
		       0,      0, 1, 0,
		       0,      0, 0, 1);

  Rtilt = create_matrix(cwt,    0,  swt, 0,
			0,      1,    0, 0,
			-swt,   0,  cwt, 0,
			0,      0,    0, 1);

  Rphi = create_matrix(1,    0,     0, 0,
		       0, cphi, -sphi, 0,
		       0, sphi,  cphi, 0,
		       0,    0,     0, 1);

  //v_erw_e = Rpan*Rtilt*Rphi*[0; 0; eh];
  v_erw_e = mat_vec_mul(mat_mat_mul(Rpan,mat_mat_mul(Rtilt,Rphi)),
			create_vector(0,0,eh));

  // we now find the position of the elbow:
  ev = vec_vec_add(e_rw, v_erw_e);

  // This lets us find the p(=theta(1)), t(=theta(2)),
  // a(=theta(3)!=phi) to the elbow:
  
  theta.j[1] = (PI/2) - atan2(ev.v[2],
			      sqrt((ev.v[0]*ev.v[0]) + (ev.v[1]*ev.v[1])));
  if(fabs(theta.j[1])<0.001){ // check if pointing straight up
    theta.j[0] = wp;
  }else{
    theta.j[0] = atan2(ev.v[1],ev.v[0]);
  }
  
  // find the rotation a, necessary to acheive this by finding
  // the point q on shoulder-wrist line that is normal from elbow
  // first find distance dq to project;

  if(eh==0){
    theta.j[2] = pos.phi;
  }else{
    angle_should_elb = atan2(eh,ehr);
    dq = DH_d[2] / cos(angle_should_elb);
    q = scal_vec_mul(dq/vec_norm(wv),wv);
    q_el = vec_vec_sub(q,ev);
    q_el_norm = scal_vec_mul(1.0/vec_norm(q_el),q_el);
    
    up_ang = theta.j[1]-(PI/2);
    perp_up_el = create_vector(sin(up_ang)*cos(theta.j[0]),
			       sin(up_ang)*sin(theta.j[0]),
			       cos(up_ang));
    perp_left_el = vec_cross(perp_up_el,ev);
    perp_left_el = scal_vec_mul(1.0/vec_norm(perp_left_el),perp_left_el);
    q_up = vec_dot(q_el_norm,perp_up_el);
    q_left = vec_dot(q_el_norm,perp_left_el);
    
    theta.j[2] = atan2(q_left,-q_up);
  }

  // We now move on to find the bend angle of the wrist, which should
  // be the angle between link 4 (between elbow and wrist) and link 7
  // (the tool), using (a/|a|)*(b/|b|) = cos(theta)

  link4 = vec_vec_sub(wv,ev);
  tool_v = mat_vec_mul(Rot,tool);

  theta.j[5]=acos(vec_dot( scal_vec_mul(1.0/vec_norm(link4),link4),
			   (scal_vec_mul(1.0/vec_norm(tool_v),tool_v))));

  // however, we need to correct for the sign, as the correct sign is
  // not found here, only magnitude. This is done further down.

  // we find theta(5) by finding the angle between link 3 (between
  // shoulder and elbow) and link 7 (the tool), as projected on the
  // plane which link 4 is normal to:
  // if theta(6) is 0, we set theta 5 to external value
  // in current implementation we just set 0
  if(fabs(theta.j[5])<0.00001) {
    theta.j[4] = t5; //theta.j[4] = t5;
  }else{

    //start by projecting link 7 to plane:
    tool_proj = vec_vec_sub(tool_v,
			scal_vec_mul(1.0/(vec_norm(link4)*vec_norm(link4)),
				     scal_vec_mul(vec_dot(tool_v,link4),link4)));
    tool_proj = scal_vec_mul(1.0/vec_norm(tool_proj),tool_proj);
    
    // project link 3 to plane and normalize:
    link3_proj = vec_vec_sub(ev,
			scal_vec_mul(1.0/(vec_norm(link4)*vec_norm(link4)),
				     scal_vec_mul(vec_dot(ev,link4),link4)));
    link3_proj = scal_vec_mul(1.0/vec_norm(link3_proj),link3_proj);

    link3_perp = vec_cross(link3_proj,link4);
    link3_perp = scal_vec_mul(1.0/vec_norm(link3_perp),link3_perp);
    
    // next find norm of parts paralell and perpendicular to
    // link3_proj in this plane:
    
    tool_par = vec_dot(tool_proj,link3_proj);
    tool_perp = vec_dot(tool_proj,link3_perp);
    
    // angle is now found using atan2 on the two parts.
    theta.j[4] = atan2(-tool_perp,tool_par);
    
  }

  // This leaves theta(7), which we find by calculating T0_6, and
  // adding the rotation needed to achieve the desired [pta]
  // hmmm... there must be a better way!

  // correct sign of theta(6) while we are at it
  
  T = generateT(0,8,theta);
  
  theta_m = theta;
  theta_m.j[5] = -theta.j[5];
  Tm = generateT(0,8,theta_m);
  
  TargetM = mat_vec_mul(Tm, tool);
  TargetP = mat_vec_mul(T,tool);
  Target = create_vector(ptx, pty, ptz);
  
  if(vec_norm(vec_vec_sub(TargetM,Target)) < 
     vec_norm(vec_vec_sub(TargetP,Target))){
    theta.j[5] = -(theta.j[5]);
    T = Tm;
  }
  Rot = T;

  // remember that:
  // r12=-cp*ct*sa - sp*ca;
  // r22=cp*ca - sp*ct*sa;
  // r32=st*sa;

  // we get
  // -cp*ct*sa - sp*ca = Rot(1,2);
  // -sp*ct*sa + cp*ca = Rot(2,2);
  // st*sa             = Rot(3,2);

  // avoid divisions with (near-) zero

  r12 = Rot.m[0][1];
  r22 = Rot.m[1][1];
  r32 = Rot.m[2][1];

  if(fabs(st)<0.01){
    if(fabs(cp)<0.00001){
      ca = (-r12)/sp;
      sa = (-r22)/(sp*ct);
    }else{
      if(fabs(sp)<0.00001){
      ca = (r12)/cp;
      sa = (-r22)/(cp*ct);
      }else{
	/*
	ca = (-(cp*ct*sa)-(r12))/sp;
	sa = (cp*ca - r22)/(sp*ct);
	  
	sa = cp/(sp*ct)*ca - r22/(sp*ct);
	sa = -(cp*cp)/(sp*sp)*sa - (cp*r12)/(sp*sp*ct) - r22/(sp*ct);
	sa = -(cp*cp)/(sp*sp)*sa - (cp*r12)/(sp*sp*ct) - r22/(sp*ct);
	(1 + (cp*cp)/(sp*sp))*sa = - (cp*r12)/(sp*sp*ct) - r22/(sp*ct);
	sa = (-(cp*r12)/(sp*sp*ct) - r22/(sp*ct))/((1 + (cp*cp)/(sp*sp)));
	*/
	sa = (-(cp*r12)/(sp*sp*ct) - r22/(sp*ct))/((1 + (cp*cp)/(sp*sp)));
	ca = (-(cp*ct*sa)-(r12))/sp;
      }
    }
  }else{
    sa = r32/st;
    if(fabs(cp)<0.01){
      ca = (-(cp*ct*sa)-(r12))/sp;
    }else{
      ca = ((sp*ct*sa)+(r22))/cp;
    }
  }

  prel_a = atan2(sa,ca);
  theta.j[6] = a-prel_a;
  while(theta.j[6]>PI){
    theta.j[6] -= (2*PI);
  }
  while(theta.j[6] < (-PI)){
    theta.j[6] += (2*PI);
  }
  return theta;
 
}

// temporarily sets the tool to be the right tool,
// calculates the inv kin, and then resets the tool value
struct joint_t inv_kin_L(struct pos_t pos, double t5){
  struct vec4 tool_temp;
  struct joint_t out_theta;
  struct pos_t pos_arm_frame;
  struct mat4 T_base, T_arm, T_base2arm_L;
  struct pos_t pos_check;
  int i;
  int ret = 0;

  // First, we change pos from base frame to 
  // the right arm's zero frame 

  // Obtains the base frame to arm (zero-) frame
  // transformation  matrix
  T_base2arm_L = inv_mat(T_arm2base_L);

  // transform pos to homog. transf. matrix
  T_base = pos_2_T(pos);
  //transform to arm zero-frame
  T_arm = mat_mat_mul(T_base2arm_L,T_base);

  // transform the matrix to pos type (struct pos_t)
  pos_arm_frame = T_2_pos(T_arm);

  // correct potential problems with t = 0;
  if(fabs(pos_arm_frame.t)<0.0001){
    pos_arm_frame.t = 0.0;
    
    pos_arm_frame.p = pos_arm_frame.p + pos_arm_frame.a;
    pos_arm_frame.a = 0.0;

    while(pos_arm_frame.p<-PI) pos_arm_frame.p += 2*PI;
    while(pos_arm_frame.p>PI) pos_arm_frame.p -= 2*PI;
  }
    

  pos_arm_frame.phi = pos.phi;
  pos_arm_frame.gripper = pos.gripper;

  tool_temp = tool;
  tool = tool_L;
  out_theta = inv_kin(pos_arm_frame,t5);
  tool = tool_temp;

  // check to make sure that the inv kin solution is correct...

  pos_check = fwd_kin_L(out_theta, 8);
  
  if(fabs(pos_check.x - pos.x)>0.0001) ret-=1;
  if(fabs(pos_check.y - pos.y)>0.0001) ret-=1;
  if(fabs(pos_check.z - pos.z)>0.0001) ret-=1;

  // **do a better check for changes in the orientation!!!
  /* if(fabs(pos_check.p - pos.p)>0.0001) ret-=1; */
  /* if(fabs(pos_check.t - pos.t)>0.0001) ret-=1; */
  /* if(fabs(pos_check.a - pos.a)>0.0001) ret-=1; */

  if(fabs(pos_check.phi - pos.phi)>0.0001) ret-=1;
    
  if(ret<0){
    printf("Error inv kin\n");
    for(i=0;i<7;i++) out_theta.j[i] = 0;
  }

  return out_theta;
}



// temporarily sets the tool to be the right tool,
// calculates the inv kin, and then resets the tool value
struct joint_t inv_kin_R(struct pos_t pos, double t5){
  struct vec4 tool_temp;
  struct joint_t out_theta;
  struct pos_t pos_arm_frame;
  struct mat4 T_base, T_arm, T_base2arm_R;
  struct pos_t pos_check;
  int i;
  int ret = 0;

  // First, we change pos from base frame to 
  // the right arm's zero frame 

  // Obtains the base frame to arm (zero-) frame
  // transformation  matrix
  T_base2arm_R = inv_mat(T_arm2base_R);

  // transform pos to homog. transf. matrix
  T_base = pos_2_T(pos);
  //transform to arm zero-frame
  T_arm = mat_mat_mul(T_base2arm_R,T_base);

  // transform the matrix to pos type (struct pos_t)
  pos_arm_frame = T_2_pos(T_arm);

  // correct potential problems with t = 0;
  if(fabs(pos_arm_frame.t)<0.0001){
    pos_arm_frame.t = 0.0;
    
    pos_arm_frame.p = pos_arm_frame.p + pos_arm_frame.a;
    pos_arm_frame.a = 0.0;

    while(pos_arm_frame.p<-PI) pos_arm_frame.p += 2*PI;
    while(pos_arm_frame.p>PI) pos_arm_frame.p -= 2*PI;
  }
    

  pos_arm_frame.phi = pos.phi;
  pos_arm_frame.gripper = pos.gripper;

  tool_temp = tool;
  tool = tool_R;
  out_theta = inv_kin(pos_arm_frame,t5);
  tool = tool_temp;

  // check to make sure that the inv kin solution is correct...

  pos_check = fwd_kin_R(out_theta, 8);
  
  if(fabs(pos_check.x - pos.x)>0.0001) ret-=1;
  if(fabs(pos_check.y - pos.y)>0.0001) ret-=1;
  if(fabs(pos_check.z - pos.z)>0.0001) ret-=1;

  // **do a better check for changes in the orientation!!!
  /* if(fabs(pos_check.p - pos.p)>0.0001) ret-=1; */
  /* if(fabs(pos_check.t - pos.t)>0.0001) ret-=1; */
  /* if(fabs(pos_check.a - pos.a)>0.0001) ret-=1; */

  if(fabs(pos_check.phi - pos.phi)>0.0001) ret-=1;
  
  if(ret<0){
    printf("Error inv kin\n");
    for(i=0;i<7;i++) out_theta.j[i] = 0;
  }

  return out_theta;
}


struct jac_t jacob_L(struct joint_t theta){

  struct jac_t jacobian;
  struct pos_t pos_e, pos;

  int i,j;
  struct mat4 T;
  struct vec4 Zi, Oe, Oi, OemOi, Jvi;
  
  // variables used for numeric calculation of 
  // the phi-row of the jacobian
  struct pos_t ref_pos;
  double ref_phi, scale, d_phi;
  double phi_row[7];
  struct joint_t d_theta;

  pos_e = fwd_kin_L(theta, 8);
	
  Oe = create_vector(pos_e.x, pos_e.y, pos_e.z);

  // Each column of the jacobian is calculated as follows: 
  // Jvi = [Zi x (Oe - Oi)];
  // Jwi = [Zi];
  // Where Jvi corresponds to the translational velocity part , 
  // and Jwi corresponds to the last 3 rows (rotational velocity)
  // For details check Robot Modeling and Control, by M. Spong.

  for(i=1;i<=7;i++){

    pos = fwd_kin_L(theta, i);
      	
    Oi = create_vector(pos.x, pos.y, pos.z);
		
    T = pos_2_T(pos);

    Zi = create_vector(T.m[0][2], T.m[1][2], T.m[2][2]);

    // Jw
    for(j=3;j<6;j++)
      {
	jacobian.J[j][i-1] = Zi.v[j-3];
      }
      
    OemOi = vec_vec_sub(Oe, Oi);
    Jvi = vec_cross(Zi, OemOi);

    // Jv
    for(j=0;j<3;j++)
      {
	jacobian.J[j][i-1] = Jvi.v[j];
      }
  }

  // calculate the last row of the jacobian numerically...
  for(i=0; i<7; i++){
    phi_row[i] = 0;
  }
  
  ref_pos = fwd_kin_L(theta, 8);
  ref_phi = ref_pos.phi;
  scale = 100000;

  for(i=0;i<4;i++){
    // set d_theta to zero
    for(j=0;j<7;j++) d_theta.j[j] = 0;

    d_theta.j[i] = 1/scale;
    for(j=0;j<7;j++) d_theta.j[j] = d_theta.j[j] + theta.j[j];

    ref_pos = fwd_kin_L(d_theta, 8);
    d_phi = (ref_pos.phi - ref_phi)*scale;

    if(fabs(d_phi) > 0.000001){
      phi_row[i] = d_phi;
    }
  }
    
  // copy phi_row to the jacobian matrix
  for(i=0;i<7;i++){
    jacobian.J[6][i] = phi_row[i];
  } 
  

  return jacobian;

}



struct jac_t jacob_R(struct joint_t theta){

  struct jac_t jacobian;
  struct pos_t pos_e, pos;

  int i,j;
  struct mat4 T;
  struct vec4 Zi, Oe, Oi, OemOi, Jvi;
  
  // variables used for numeric calculation of 
  // the phi-row of the jacobian
  struct pos_t ref_pos;
  double ref_phi, scale, d_phi;
  double phi_row[7];
  struct joint_t d_theta;

  pos_e = fwd_kin_R(theta, 8);
	
  Oe = create_vector(pos_e.x, pos_e.y, pos_e.z);

  // Each column of the jacobian is calculated as follows: 
  // Jvi = [Zi x (Oe - Oi)];
  // Jwi = [Zi];
  // Where Jvi corresponds to the translational velocity part , 
  // and Jwi corresponds to the last 3 rows (rotational velocity)
  // For details check Robot Modeling and Control, by M. Spong.

  for(i=1;i<=7;i++){

    pos = fwd_kin_R(theta, i);
      	
    Oi = create_vector(pos.x, pos.y, pos.z);
		
    T = pos_2_T(pos);

    Zi = create_vector(T.m[0][2], T.m[1][2], T.m[2][2]);

    // Jw
    for(j=3;j<6;j++)
      {
	jacobian.J[j][i-1] = Zi.v[j-3];
      }
      
    OemOi = vec_vec_sub(Oe, Oi);
    Jvi = vec_cross(Zi, OemOi);

    // Jv
    for(j=0;j<3;j++)
      {
	jacobian.J[j][i-1] = Jvi.v[j];
      }
  }

  // calculate the last row of the jacobian numerically...
  for(i=0; i<7; i++){
    phi_row[i] = 0;
  }
  
  ref_pos = fwd_kin_R(theta, 8);
  ref_phi = ref_pos.phi;
  scale = 100000;

  for(i=0;i<4;i++){
    // set d_theta to zero
    for(j=0;j<7;j++) d_theta.j[j] = 0;

    d_theta.j[i] = 1/scale;
    for(j=0;j<7;j++) d_theta.j[j] = d_theta.j[j] + theta.j[j];

    ref_pos = fwd_kin_R(d_theta, 8);
    d_phi = (ref_pos.phi - ref_phi)*scale;

    if(fabs(d_phi) > 0.000001){
      phi_row[i] = d_phi;
    }
  }
    
  // copy phi_row to the jacobian matrix
  for(i=0;i<7;i++){
    jacobian.J[6][i] = phi_row[i];
  } 
  

  return jacobian;

}


struct vel_screw_t fwd_vel(struct jac_t jacobian, struct joint_t vel){

  struct vel_screw_t vel_screw;
  int i,j;

  for(i=0; i<3; i++) vel_screw.v[i] = 0;
  for(i=0; i<3; i++) vel_screw.w[i] = 0;
  vel_screw.w_phi = 0;

  for(i=0; i<3; i++){
    for(j=0; j<7; j++){
      vel_screw.v[i] += jacobian.J[i][j]*vel.j[j];
    }
  }

  for(i=3; i<6; i++){
    for(j=0; j<7; j++){
      vel_screw.w[i-3] += jacobian.J[i][j]*vel.j[j];
    }
  }

  for(j=0; j<7; j++){
      vel_screw.w_phi += jacobian.J[6][j]*vel.j[j];    
  }

  return vel_screw;
}


// Calculates the inverse jacobian matrix
struct jac_t inv_jacob(struct jac_t jacobian){
  
  struct jac_t _C;
  struct jac_t _I;
  struct jac_t _Iout;
  int _ri[7]; // row index
  int i,j,k;
  double _max_val=0.0;
  int _max_pos = 0;
  int _tmp;
  double _temp_div;
  double _temp_mul;


  // Copy input and generate identity matrix
  for(i=0;i<7;i++){
    for(j=0;j<7;j++){
      _C.J[i][j] =  (double)jacobian.J[i][j];
      _I.J[i][j] = (double)(i==j);
    }
    _ri[i]=i;
  }
  
  for(i=0;i<7;i++){
    
    // find largest element in column i
    _max_val = 0.0;
    for(j=i;j<7;j++){
      if(fabs(_C.J[_ri[j]][i])>_max_val){
        _max_val = fabs(_C.J[_ri[j]][i]);
        _max_pos = j;
      }
    }

    //switch rows:
    _tmp=_ri[i];
    _ri[i]=_ri[_max_pos];
    _ri[_max_pos]=_tmp;

    // normalize row i
    _temp_div = _C.J[_ri[i]][i];
    for(j=0;j<7;j++){
      _I.J[_ri[i]][j] /= _temp_div;
      _C.J[_ri[i]][j] /= _temp_div;
    }
    // eliminate lower rows
    if(i<6){
      for(j=i+1; j<7; j++){
        _temp_mul =  _C.J[_ri[j]][i];
        for(k=0; k<7; k++){
          _I.J[_ri[j]][k] -= _temp_mul*_I.J[_ri[i]][k];
          _C.J[_ri[j]][k] -= _temp_mul*_C.J[_ri[i]][k];
        }
      }
    }
    
  }
 
  // eliminate upper rows
  for(i=5; i>(-1); i--){
    for(j=0; j<(i+1); j++){
      for(k=0; k<7; k++){
        _I.J[_ri[j]][k] -= _C.J[_ri[j]][i+1]*_I.J[_ri[i+1]][k];
      }
    }
  }

  for(i=0; i<7; i++){
    for(j=0; j<7; j++){
      if(isnan(_Iout.J[i][j] = (float)_I.J[_ri[i]][j])){
	fprintf(stderr,"WARNING: NaN in matrix after inversion!\n");
      }
    }
  }


  return _Iout;
}

struct joint_t inv_vel(struct jac_t inv_jacobian, struct vel_screw_t vel_screw){


  struct joint_t vel;
  int i,j; 

  for(i=0;i<7;i++) vel.j[i] = 0;
    

  for(i=0; i<7; i++){

    for(j=0; j<3; j++){
      vel.j[i] += inv_jacobian.J[i][j]*vel_screw.v[j];
    }

    for(j=0; j<3; j++){
      vel.j[i] += inv_jacobian.J[i][j+3]*vel_screw.w[j];
    }
    
    vel.j[i] += inv_jacobian.J[i][6]*vel_screw.w_phi;
    
  }
  
  return vel;

}


// transforms a pos structure 
// to a homogenic transformation matrix
struct mat4 pos_2_T(struct pos_t pos){

  struct mat4 T;
  double r11, r12, r13;
  double r21, r22, r23;
  double r31, r32, r33;
  double p,t,a;
  double cp, sp, ct, st, ca, sa;

  p = pos.p;
  t = pos.t;
  a = pos.a;  
    
  // precalculate values often needed:
  sp=sin(p);
  st=sin(t);
  sa=sin(a);
  cp=cos(p);
  ct=cos(t);
  ca=cos(a);

  //create rotation matrix:
  r13=cp*st;
  r23=sp*st;
  r33=ct;
  r12=-cp*ct*sa - sp*ca;
  r22=cp*ca - sp*ct*sa;
  r32=st*sa;

  // calculate the last column from the cross product
  r11 = r22*r33 - r23*r32;
  r21 = r32*r13 - r12*r33;
  r31 = r12*r23 - r13*r22;

  // Generate rotational matrix
  T = create_matrix(r11, r12, r13, pos.x,
		    r21, r22, r23, pos.y,
		    r31, r32, r33, pos.z,
		     0,   0,   0,    1  );

  return T;

}

struct pos_t T_2_pos(struct mat4 T){

  struct pos_t pos, pos_temp; 

  pos.x = T.m[0][3];
  pos.y = T.m[1][3];
  pos.z = T.m[2][3];

  pos_temp = T_2_euler(T);
  
  pos.p = pos_temp.p;
  pos.t = pos_temp.t;
  pos.a = pos_temp.a;

  return pos; 

}


// returns ZYZ-euler angles that represent
// the rotation matrix contained in
// the homogenic transformation matrix T. 
// The angles are placed in the pos structure.
struct pos_t T_2_euler(struct mat4 T){

  double r11, r12, r13;
  double r21, /*r22,*/ r23;
  double r31, r32, r33;

  struct pos_t pos;

  pos.x = 0;
  pos.y = 0;
  pos.z = 0;
  pos.phi = 0;
 
  r11 = T.m[0][0];
  r12 = T.m[0][1];
  r13 = T.m[0][2];
  r21 = T.m[1][0];
  /* r22 = T.m[1][1]; */ // not needed
  r23 = T.m[1][2];
  r31 = T.m[2][0];
  r32 = T.m[2][1];
  r33 = T.m[2][2];


  if((fabs(r13)>=0.005)||(fabs(r23)>=0.005)){
    pos.t = atan2(sqrt(1-r33*r33),r33);
    pos.p = atan2(r23,r13);
    pos.a = atan2(r32,-r31);
  }

  else{
    if(fabs(r33-1)<=0.005){
      pos.t = 0;
      pos.p = 0;
      pos.a = atan2(r21,r11);
    }

    else{
      pos.t = PI;
      pos.p = 0;
      pos.a = -atan2(-r12,-r11);
    }   

  }

  return pos;
}



struct mat4 T_2_rot(struct mat4 T){

  int i;
  struct mat4 R;
  
  R = T;
  for(i=0;i<3;i++) R.m[i][3] = 0.0;

  return R;
}


double fwd_kin_phi(struct joint_t theta){

  struct vec4 zero_vec, wv, ev;
  struct vec4 up, yv, xv;
  struct mat4 T4, T7;
  double ey, ex, phi;

  zero_vec = create_vector(0, 0, 0);  
  T4 = generateT(0, 4, theta);
  T7 = generateT(0, 7, theta);

  // wrist vector
  wv = mat_vec_mul(T7, zero_vec);
  
  // find if wrist isnt straight up, so phi is ill defined
  if((vec_norm(wv) - fabs(wv.v[2])) > vec_norm(wv)/1000){

    //find if elbow is straight, and phi = theta(3)
    if(fabs(theta.j[3])<0.0001){
      phi = theta.j[2];
    }

    else{

      // elbow vector
      ev = mat_vec_mul(T4, zero_vec);
    
      // up vector;
      up = create_vector(0, 0, 1);

      // y-vector in normal plane of wv
      yv = vec_cross(wv, up);
      yv = vec_unit(yv);

      // x-vector in normal plane of wv
      xv = vec_cross(yv, wv);
      xv = vec_unit(xv);

      // x-component of elbow position
      ex = vec_dot(xv, ev);
    
      // y-component of elbow position
      ey = vec_dot(yv, ev);

      phi = atan2(ey, ex);
    
    }
  }

  // phi ill-defined
  else{

    phi = theta.j[0];
    
    if(fabs(theta.j[3])<0.0001){
      phi += theta.j[2];
    }

  }

  return phi;

}

void vel_set_zero(struct joint_t *joint_vel){

  int i;
  
  for(i=0;i<7;i++){
    joint_vel->j[i] = 0.0;
  }

  joint_vel->gripper = 0.0;

}


void print_pos(struct pos_t pos){
  fprintf(stdout, "x: %6.4f m\n", pos.x);
  fprintf(stdout, "y: %6.4f m\n", pos.y);
  fprintf(stdout, "z: %6.4f m\n", pos.z);
  fprintf(stdout, "p: %6.4f rad, %8.4f deg\n", pos.p, pos.p*180/PI);
  fprintf(stdout, "t: %6.4f rad, %8.4f deg\n", pos.t, pos.t*180/PI);
  fprintf(stdout, "a: %6.4f rad, %8.4f deg\n", pos.a, pos.a*180/PI);

  fprintf(stdout, "phi: %6.4f rad, %8.4f deg\n", pos.phi, pos.phi*180/PI);

  fprintf(stdout, "Gripper: %6.4f m\n", pos.gripper);
}


void print_joint(struct joint_t theta){
  int i;
  for(i=0;i<7;i++){
    fprintf(stdout, "j%d: %6.4f rad, %8.4f deg\n",
	    i, theta.j[i], theta.j[i]*180/PI);
  }
  fprintf(stdout, "Gripper: %6.4f m\n", theta.gripper);
}

void print_joint_vel(struct joint_t vel){
 int i;
  for(i=0;i<7;i++){
    fprintf(stdout, "j%d: %6.4f rad/s, %8.4f deg/s\n",
	    i, vel.j[i], vel.j[i]*180/PI);
  }
  fprintf(stdout, "Gripper: %6.4f m/s\n", vel.gripper);
}


void print_jacob(struct jac_t jacobian){

  int i,j;

  for(i=0;i<7;i++){
    fprintf(stdout,"| ");
    for(j=0;j<7;j++){
      fprintf(stdout,"%6.4f ",jacobian.J[i][j]);
    }
    fprintf(stdout,"|\n");
  }
}

void print_vel_screw(struct vel_screw_t vel_screw){

  int i;
  for(i=0;i<3;i++){
    fprintf(stdout, "v(%d): %6.4f m/s\n",
	    i, vel_screw.v[i]);
  }

  for(i=0;i<3;i++){
    fprintf(stdout, "w(%d): %6.4f rad/s %8.4f deg/s\n",
	    i, vel_screw.w[i], vel_screw.w[i]*180/PI);
  }

  fprintf(stdout, "w_phi: %6.4f\n", vel_screw.w_phi);


}


