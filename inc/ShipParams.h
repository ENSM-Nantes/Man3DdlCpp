#ifndef SHIP_PARAMS_H
#define SHIP_PARAMS_H

#include <iostream>
#include <vector>

typedef struct{
  double lPP;
  double b;
  double d;
  double volume;
  double xG;
  double cB;
}sGeoParams;

typedef struct{
  double xp0;
  double xpVV;
  double xpVR;
  double xpRR;
  double xpVVVV;
  double ypV;
  double ypR;
  double ypVVV;
  double ypVVR;
  double ypVRR;
  double ypRRR;
  double npV;
  double npR;
  double npVVV;
  double npVVR;
  double npVRR;
  double npRRR;
}sHullDerParams;

typedef struct{
  double mpX;
  double mpY;
  double jpZ;
}sAddedMassParams;

typedef struct{
  double dP;
  double tP;
  double xpP;
  double wP0;
  double k0;
  double k1;
  double k2;
  double nP;
}sPropellerParams;

typedef struct{
  double hR;
  double aR;
  double xpR;
  double aH;
  double tR;
  double xpH;
  double epsilon;
  double kappa;
  double lpR;
  double lambdaR;
  std::vector<double> gammaR;
  double rrMax;
}sRudderParams;

typedef struct{
  double aL;
  double aT;
  double aSS;
  double S;
  double C;
  double M;
  double rho;
  double velWind;
  double psiWind;
}sShipWindParams;

#endif
