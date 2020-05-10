#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "../src/Footy/constantes.h"

#define EPUCK2DEG(a)				((float)(a)*ANGULAR_UNIT)

int main(void) {
  FILE * fp;
  int16_t distance;
  float correction_factor;
  fp = fopen ("lookup_ball_angle_dist.txt","w");

  //generate arcos values in tab
  fprintf(fp, "{");
  for(int16_t i = 0; i < N_PRECALCULATED_ANGLE_TO_DIST_VALUES; ++i)
  {
	distance = BALL_DIAMETER/2/sin((EPUCK2DEG(MIN_HALF_ANGLE_BALL)+i*EPUCK2DEG(ANGLE_TO_DIST_ANGLE_RES))*M_PI/180);
	correction_factor=1.583-0.0004f*distance+0.00000004f*distance*distance;
	distance = correction_factor * distance;
    fprintf(fp, "%d", distance);
    if(i!=N_PRECALCULATED_ANGLE_TO_DIST_VALUES-1)
    {
      fprintf(fp, ",\t");
	  if(distance < 100.f)
		fprintf(fp, ",\t");
      if(i%10==9)
        fprintf(fp, "\n");
    }
    else
    {
	fprintf(fp, "}\n");
    }
  }

  return 0;
}