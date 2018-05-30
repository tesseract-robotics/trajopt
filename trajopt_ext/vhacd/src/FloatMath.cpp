#include "FloatMath.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#define REAL float

#include "FloatMath.inl"

#undef REAL
#define REAL double

#include "FloatMath.inl"
