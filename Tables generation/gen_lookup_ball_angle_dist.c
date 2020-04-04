#include <stdio.h>
#include <math.h>

int main(void) {
  float distUnit, ballRadius, fullAngle, angleStep, nDivision;
  FILE * fp;
  fp = fopen ("lookup_ball_angle_dist.txt","w");

  printf("Unite de distance en mm\n");
  scanf("%f", &distUnit);
  printf("Rayon de la balle en mm\n");
  scanf("%f", &ballRadius);
  printf("Nombre d'unite d'angle dans une revolution\n");
  scanf("%f", &fullAngle);
  printf("Incrément d'angle par division (unite du programme)\n");
  scanf("%f", &angleStep);
  printf("Nombre de division\n");
  scanf("%f", &nDivision);

  //generate arcos values in tab
  fprintf(fp, "{");
  for(int i = 1; i <= nDivision; i++)
  {
    fprintf(fp, "%.0f", ballRadius/sin(i*angleStep/fullAngle*2*M_PI)/distUnit);
    if(i!=nDivision)
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