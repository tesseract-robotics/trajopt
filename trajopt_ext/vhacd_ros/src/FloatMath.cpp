#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include "vhacd_ros/inc/FloatMath.h"
#include <vector>

#define REAL float

#include "FloatMath.inl"

#undef REAL
#define REAL double

#include "FloatMath.inl"
