#include "Propeller.h"
#include <cmath>

Propeller::Propeller(void)
{
  mT << 0, 0, 0;
}

void Propeller::ComputeT(const Eigen::Vector3d& aMu, const double aRho, const sGeoParams& aGeo, const sPropellerParams& aProp)
{
  double u = 0, beta = 0, rp = 0;
  double betap = 0, wp = 0, up = 0, jp = 0;
  double kt = 0, tp = 0, xp = 0; 

  u = pow((pow(aMu[0], 2) + pow(aMu[1], 2)), 0.5);
  beta = atan(-(aMu[1])/aMu[0]);
  rp = (aMu[2] * aGeo.lPP)/u;
  betap = beta - (aProp.xpP * rp);
  
  wp = aProp.wP0 * exp(-4 * pow(betap, 2));
  up = aMu[0] * (1-wp);
  jp = up / (aProp.nP * aProp.dP);
  kt = aProp.k0 + (aProp.k1*jp) + (aProp.k2*pow(jp, 2));
  tp = aRho * pow(aProp.nP, 2) * pow(aProp.dP, 4) * kt;
  xp = (1-aProp.tP) * tp;

  mT << xp, 0, 0;

  //std::cout << "***Propeller mT :" << mT << std::endl; 
  return;
}

Eigen::Vector3d& Propeller::getPropellerT(void){return mT;}
  
