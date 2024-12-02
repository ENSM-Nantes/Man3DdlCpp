#include "Rudder.h"
#include <cmath>

Rudder::Rudder()
{
  mT << 0, 0, 0;
  mDelta = 0;
}

void Rudder::SetDelta(const double aDelta, const double aDt, const sRudderParams& aRudder)
{
  double rrSet = 0;
  rrSet = (aDelta - mDelta) / aDt;

  if(abs(rrSet) > aRudder.rrMax) mDelta += ((rrSet/abs(rrSet)) * aRudder.rrMax * aDt);
  else mDelta = aDelta;
}

void Rudder::ComputeT(const Eigen::Vector3d& aMu, const double aRho, const sGeoParams& aGeo, const sRudderParams& aRudder, const sPropellerParams& aProp)
{
  double u = 0, beta = 0, rp = 0, lambdar = 0;
  double betap = 0, wp = 0, up = 0, jp = 0;
  double kt = 0, eta = 0, tmpur = 0, ur = 0;
  double betar = 0, gammar = 0, vr = 0, alphar = 0;
  double falpha = 0, Fn =0, xr = 0, xh = 0, Xr = 0, Yr = 0, Nr = 0;
  
  u = pow((pow(aMu[0], 2) + pow(aMu[1], 2)), 0.5);
  beta = atan(-(aMu[1])/aMu[0]);
  rp = (aMu[2] * aGeo.lPP)/u;
  betap = beta - (aProp.xpP * rp);
  
  wp = aProp.wP0 * exp(-4 * pow(betap, 2));
  up = aMu[0] * (1-wp);
  jp = up / (aProp.nP * aProp.dP);
  kt = aProp.k0 + (aProp.k1*jp) + (aProp.k2*pow(jp, 2));
  
  eta = aProp.dP / aRudder.hR;
  tmpur = 1 + aRudder.kappa * (pow(1 + (8*kt/(M_PI*pow(jp, 2)) ), 0.5) - 1);
  ur = aRudder.epsilon * up * pow((eta * pow(tmpur, 2) + (1-eta)), 0.5);
  betar = beta - (aRudder.lpR * rp);

  if(betar < 0) gammar = aRudder.gammaR[0];
  else gammar = aRudder.gammaR[1];

  vr = u * gammar * betar;
  alphar = mDelta - atan(vr/ur);
  ur = pow((pow(ur, 2) + pow(vr, 2)), 0.5);

  if(0 != aRudder.lambdaR) lambdar = aRudder.lambdaR;
  else lambdar = pow(aRudder.hR, 2)/aRudder.aR;

  falpha = 6.13 * lambdar / (lambdar + 2.25);
  Fn = 0.5 * aRho * aRudder.aR * pow(ur, 2) * falpha * sin(alphar);
  xr = aRudder.xpR * aGeo.lPP;
  xh = aRudder.xpH * aGeo.lPP;

  Xr = -(1 - aRudder.tR) * Fn * sin(mDelta);
  Yr = -(1 + aRudder.aH) * Fn * cos(mDelta);
  Nr = -(xr + aRudder.aH * xh) * Fn * cos(mDelta);

  mT << Xr, Yr, Nr;
  
  //std::cout << "****Rudder mT :" << mT << std::endl; 
  return;
}

Eigen::Vector3d& Rudder::getRudderT(void){return mT;}

double Rudder::getDelta(void){return mDelta;}
