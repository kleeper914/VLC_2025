#ifndef RSS_C
#define RSS_C

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct
{             
	float x;                //x坐标
	float y;                //y坐标
}Point;

Point threePoints(float *dis, Point *ps);

Point getPDlocation(float* distaces, Point *points);

#endif // RSS_C