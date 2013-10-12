// HERE DEFINE (STRICTER) JOINT LIMITS...
// Left arm
#include "utils.h"

#define MIN_J1_L -PI
#define MIN_J2_L -120*PI/180
#define MIN_J3_L -PI
#define MIN_J4_L -120*PI/180
#define MIN_J5_L -PI
#define MIN_J6_L -120*PI/180
#define MIN_J7_L -PI
#define MIN_G_L   0

#define MAX_J1_L PI
#define MAX_J2_L 120*PI/180
#define MAX_J3_L PI
#define MAX_J4_L 120*PI/180
#define MAX_J5_L PI
#define MAX_J6_L 120*PI/180
#define MAX_J7_L PI
#define MAX_G_L  0.06

// Right arm
#define MIN_J1_R -PI
#define MIN_J2_R -120*PI/180
#define MIN_J3_R -PI
#define MIN_J4_R -120*PI/180
#define MIN_J5_R -PI
#define MIN_J6_R -120*PI/180
#define MIN_J7_R -PI
#define MIN_G_R   0

#define MAX_J1_R PI
#define MAX_J2_R 120*PI/180
#define MAX_J3_R PI
#define MAX_J4_R 120*PI/180
#define MAX_J5_R PI
#define MAX_J6_R 120*PI/180
#define MAX_J7_R PI
#define MAX_G_R  0.06

// HERE DEFINE CARTESIAN LIMITS... ***CAREFUL WITH THESE CHANGES

// Left arm
#define MIN_X_L -0.6
#define MIN_Y_L  0.4//-0.15
#define MIN_Z_L -0.5

#define MAX_X_L -0.0
#define MAX_Y_L 0.9995
#define MAX_Z_L 0.5

// Right arm
#define MIN_X_R 0.1
#define MIN_Y_R 0.4
#define MIN_Z_R -0.5

#define MAX_X_R 0.6
#define MAX_Y_R 0.9995
#define MAX_Z_R 0.5

// indexes for cartesian box

#define min_x    0 
#define min_y    1
#define min_z    2

#define max_x    3
#define max_y    4
#define max_z    5
