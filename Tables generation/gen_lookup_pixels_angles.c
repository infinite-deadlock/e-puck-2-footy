#include <stdio.h>
#include <math.h>

int main(void) {
  int imageWidth, fullRevolution;
  FILE * fp;
  fp = fopen ("gen_lookup_pixels_angles.txt","w");

  printf("Largeur de l'image en pixel\n");
  scanf("%d", &imageWidth);
  printf("Nombre de divisions dans un tour\n");
  scanf("%d", &fullRevolution);

  //generate arcos values in tab
  fprintf(fp, "{");
  int i = 1;
  for(; i <= imageWidth/2; i++)
  {
    fprintf(fp, "%.0f", atan(tan((M_PI/8))*2*i/imageWidth)*fullRevolution/2/M_PI);
    if(i!=imageWidth/2)
    {
      fprintf(fp, ",\t");
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