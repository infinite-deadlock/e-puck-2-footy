#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "../src/Footy/constantes.h"

#define TAN_45_OVER_2_CONST             0.4142135679721832275390625f // in rad, fit for float

int main(void) {
  int imageWidth;
  int16_t distance;
  FILE * fp;
  fp = fopen ("gen_lookup_pixels_angles.txt","w");

  printf("Largeur de l'image en pixel\n");
  scanf("%d", &imageWidth);

  //generate arcos values in tab
  fprintf(fp, "{");
  
  for(int i = 1; i <= imageWidth/2; i++)
  {
	distance = DEG2EPUCK(atan((float)i / (imageWidth/2) * TAN_45_OVER_2_CONST) * 180.f / M_PI);
    fprintf(fp, "%d", distance);
    if(i!=imageWidth/2)
    {
      fprintf(fp, ",\t");
	  if(distance < 100)
		fprintf(fp, "\t");
      if(i%10==0)
        fprintf(fp, "\n");
    }
    else
    {
	fprintf(fp, "}\n");
    }
  }

  return 0;
}