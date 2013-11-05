/*
 * homogenic_vectors
 *
 *  Created on: 2012
 *  Authors:   Christian Smith
 *            ccs <at> kth.se 
 */

/* Copyright (c) 2012, Christian Smith, KTH
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

// This contains some basics for doing homogenig vector transforms

#include <dumbo_powercube_chain/homogenic_vectors.h>

// returns matrix
struct mat4 create_matrix(double a00, double a01, double a02, double a03,
			  double a10, double a11, double a12, double a13,
			  double a20, double a21, double a22, double a23,
			  double a30, double a31, double a32, double a33){
  struct mat4 ret = {  .m = {{a00, a01, a02, a03},
			     {a10, a11, a12, a13},
			     {a20, a21, a22, a23},
			     {a30, a31, a32, a33}}};
  return ret;


}

// returns vector
struct vec4 create_vector(double a0, double a1, double a2){
  struct vec4 ret = {  .v = {a0, a1, a2, 1.0}};
  return ret;

}


inline struct mat4 EYE4(void){
  struct mat4 EYE = {.m = {{1,  0,  0,  0},
			   {0,  1,  0,  0},
			   {0,  0,  1,  0},
			   {0,  0,  0,  1}}};
  return EYE;
}

struct mat4 inv_mat(struct mat4 A){
  struct mat4 _C;
  struct mat4 _I;
  int _ri[4]; // row index
  int i,j,k;
  double _max_val=0.0;
  int _max_pos = 0;
  int _tmp;
  double _temp_div;
  double _temp_mul;
  struct mat4 _Iout;

  // Copy input and generate identity matrix
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      _C.m[i][j] =  (double)A.m[i][j];
      _I.m[i][j] = (double)(i==j);
    }
    _ri[i]=i;
  }
  
  for(i=0;i<4;i++){
    
    // find largest element in column i
    _max_val = 0.0;
    for(j=i;j<4;j++){
      if(fabs(_C.m[_ri[j]][i])>_max_val){
        _max_val = fabs(_C.m[_ri[j]][i]);
        _max_pos = j;
      }
    }

    //switch rows:
    _tmp=_ri[i];
    _ri[i]=_ri[_max_pos];
    _ri[_max_pos]=_tmp;

    // normalize row i
    _temp_div = _C.m[_ri[i]][i];
    for(j=0;j<4;j++){
      _I.m[_ri[i]][j] /= _temp_div;
      _C.m[_ri[i]][j] /= _temp_div;
    }
    // eliminate lower rows
    if(i<3){
      for(j=i+1; j<4; j++){
        _temp_mul =  _C.m[_ri[j]][i];
        for(k=0; k<4; k++){
          _I.m[_ri[j]][k] -= _temp_mul*_I.m[_ri[i]][k];
          _C.m[_ri[j]][k] -= _temp_mul*_C.m[_ri[i]][k];
        }
      }
    }
    
  }
 
  // eliminate upper rows
  for(i=2; i>(-1); i--){
    for(j=0; j<(i+1); j++){
      for(k=0; k<4; k++){
        _I.m[_ri[j]][k] -= _C.m[_ri[j]][i+1]*_I.m[_ri[i+1]][k];
      }
    }
  }

  for(i=0; i<4; i++){
    for(j=0; j<4; j++){
      if(isnan(_Iout.m[i][j] = (float)_I.m[_ri[i]][j])){
	fprintf(stderr,"WARNING: NaN in matrix after inversion!\n");
      }
    }
  }

  return _Iout;
}

// returns A*B
struct mat4 mat_mat_mul(struct mat4 A, struct mat4 B){
  int i,j,k;
  struct mat4 ret;
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      ret.m[i][j] = 0;
      for(k=0;k<4;k++){
	ret.m[i][j] += A.m[i][k]*B.m[k][j];
      }
    }   
  }
  return ret;
}


// returns A*b
struct vec4 mat_vec_mul(struct mat4 A, struct vec4 b){
  int i,j;
  struct vec4 ret;
  for(i=0;i<4;i++){
    ret.v[i] = 0;
    for(j=0;j<4;j++){
      ret.v[i] += A.m[i][j]*b.v[j];
    }
  }   
  return ret;
}

// returns a*b (treated as 3D vectors)
double vec_dot(struct vec4 a, struct vec4 b){
  int i;
  double ret;
  ret = 0;
  for(i=0;i<3;i++){
    ret += a.v[i]*b.v[i];
  }
  return ret;
}


// returns axb
struct vec4 vec_cross(struct vec4 a, struct vec4 b){
  struct vec4 ret = {.v = {a.v[1]*b.v[2]-a.v[2]*b.v[1],
			   a.v[2]*b.v[0]-a.v[0]*b.v[2],
			   a.v[0]*b.v[1]-a.v[1]*b.v[0]}};
  return ret;
}

 
// prints a vector
void print_vec(struct vec4 a){
  int i;
  for(i=0;i<4;i++){
    fprintf(stdout,"| %6.4f |\n",a.v[i]);
  }
}

 // prints a matrix
void print_mat(struct mat4 A){
  int i,j;
  for(i=0;i<4;i++){
    fprintf(stdout,"| ");
    for(j=0;j<4;j++){   
       fprintf(stdout,"%6.4f ",A.m[i][j]);
    }
    fprintf(stdout,"|\n");
  }
}

// returns a+b (treated as 3D vectors)
struct vec4 vec_vec_add(struct vec4 a, struct vec4 b){
  struct vec4 ret;
  int i;
  for(i=0;i<3;i++){
    ret.v[i] = a.v[i]+b.v[i];
  }
  ret.v[3] = 1.0;
  return ret;
}

// returns a-b (treated as 3D vectors)
struct vec4 vec_vec_sub(struct vec4 a, struct vec4 b){
  struct vec4 ret;
  int i;
  for(i=0;i<3;i++){
    ret.v[i] = a.v[i]-b.v[i];
  }
  ret.v[3] = 1.0;
  return ret;
}

// returns n*a (treated as scalar and 3D vector)
struct vec4 scal_vec_mul(double n, struct vec4 a){
  struct vec4 ret;
  int i;
  for(i=0;i<3;i++){
    ret.v[i] = a.v[i]*n;
  }
  ret.v[3] = 1.0;
  return ret;
}


// returns norm(a) (treated as 3D vector)
double vec_norm(struct vec4 a){
  double ret = 0.0;
  int i;
  for(i=0;i<3;i++){
    ret += a.v[i]*a.v[i];
  }
  ret = sqrt(ret);
  return ret;
}


// normalizes a vector so that its norm is 1
struct vec4 vec_unit(struct vec4 a){

  double norm;
  struct vec4 b = create_vector(0, 0, 0);

  norm = vec_norm(a);
  
  if(fabs(norm)>0.0001)b = scal_vec_mul(1/norm, a);

  return b;

}
