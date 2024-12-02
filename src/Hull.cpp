#include "Hull.h"
#include <cmath>

Hull::Hull(void)
{
  mT << 0, 0, 0;
}

void Hull::ComputeT(const Eigen::Vector3d& aMu, const double aRho, const sGeoParams& aGeo, const sHullDerParams& aHull)
{
  double u = 0, kf = 0, km = 0, vp = 0;
  double rp = 0, xph = 0, yph = 0, nph = 0;

  u = pow((pow(aMu[0], 2) + pow(aMu[1], 2)), 0.5);
  kf = 0.5 * aRho * aGeo.lPP * aGeo.d * pow(u , 2);
  km = 0.5 * aRho * pow(aGeo.lPP, 2) * aGeo.d * pow(u , 2);
  vp = aMu[1] / u;
  rp = aMu[2] * aGeo.lPP/u;

  xph = -aHull.xp0 + (aHull.xpVV * pow(vp, 2)) + (aHull.xpVR * vp * rp) + (aHull.xpRR * pow(rp, 2)) + (aHull.xpVVVV * pow(vp, 4));
  
  yph = (aHull.ypV * vp) + (aHull.ypR * rp) + (aHull.ypVVV * pow(vp, 3)) + (aHull.ypVVR * pow(vp, 2) * rp) + (aHull.ypVRR * pow(rp, 2) * vp) + (aHull.ypRRR * pow(rp, 3));

  nph = (aHull.npV * vp) + (aHull.npR * rp) + (aHull.npVVV * pow(vp, 3)) + (aHull.npVVR * pow(vp, 2) * rp) + (aHull.npVRR * pow(rp, 2) * vp) + (aHull.npRRR * pow(rp, 3));

  mT << kf*xph, kf*yph, km*nph;

  //std::cout << "****Hull mT :" << mT << std::endl; 
  return;
}

Eigen::Vector3d& Hull::getHullT(void){return mT;}
