#include "tools.h"
#include <cmath>

int toolsSign(double x)
{
  if(x > 0) return 1;
  else if(x < 0) return -1;
  else return 1;
}

double toolsDegToRad(double a)
{
  return a*M_PI/180;
}

double toolsRadToDeg(double a)
{
  return a*180/M_PI;
}
