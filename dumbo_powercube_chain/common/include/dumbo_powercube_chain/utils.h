// Header file containing some common utility defines


#ifndef _utils_h_

#define _utils_h_

#ifndef PI
#define PI (3.141592653589793)
#endif

#ifndef SIGN
#define SIGN(X) ((X)>=(0)?(1):(-1))
#endif

#ifndef MAX
#define MAX(X,Y) ((X)>(Y)?(X):(Y))
#endif

#ifndef MIN
#define MIN(X,Y) ((X)<(Y)?(X):(Y))
#endif

#endif
