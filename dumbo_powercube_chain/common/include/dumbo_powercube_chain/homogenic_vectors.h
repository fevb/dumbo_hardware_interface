#ifndef _homogenic_vectors_h_
#define _homogenic_vectors_h_

// Some basics for doing homogenic vector stuff

#include <stdio.h>
#include <math.h>

#define PRINT_VEC(A) fprintf(stdout, #A ":\n"); print_vec((A))
#define PRINT_MAT(A) fprintf(stdout, #A ":\n"); print_mat((A))
#define PRINT_DOUBLE(A) fprintf(stdout, #A ": %f\n", (A))

struct vec4{
  double v[4];
};

struct mat4{
  double m[4][4];
};


// returns matrix
struct mat4 create_matrix(double a00, double a01, double a02, double a03,
			  double a10, double a11, double a12, double a13,
			  double a20, double a21, double a22, double a23,
			  double a30, double a31, double a32, double a33);

// returns vector with 4th element set to 1
struct vec4 create_vector(double a0, double a1, double a2);


// returns 4x4 identity matrix
inline struct mat4 EYE4(void);

// returns inv(A)
struct mat4 inv_mat(struct mat4 A);

// returns A*B
struct mat4 mat_mat_mul(struct mat4 A, struct mat4 B);

// returns A*b
struct vec4 mat_vec_mul(struct mat4 A, struct vec4 b);

// returns a+b (treated as 3D vectors)
struct vec4 vec_vec_add(struct vec4 a, struct vec4 b);

// returns a-b (treated as 3D vectors)
struct vec4 vec_vec_sub(struct vec4 a, struct vec4 b);

// returns n*a (treated as scalar and 3D vector)
struct vec4 scal_vec_mul(double n, struct vec4 a);

// returns norm(a) (treated as 3D vector)
double vec_norm(struct vec4 a);

// normalizes a vector so that its norm is 1
struct vec4 vec_unit(struct vec4 a);



// returns a*b
double vec_dot(struct vec4 a, struct vec4 b);

// returns axb
struct vec4 vec_cross(struct vec4 a, struct vec4 b);

// prints a vector
void print_vec(struct vec4 a);

// prints a matrix
void print_mat(struct mat4 A);


#endif
