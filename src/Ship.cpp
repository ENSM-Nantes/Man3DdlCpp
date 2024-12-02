#include <cmath>
#include "tools.h"
#include "Ship.h"

Ship::Ship()
{
  mRho = 0;
  mM = 0;
  mMX = 0;
  mMY = 0;
  mMatM << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  mInvMatM << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  mMu << 0, 0, 0;
  mEta << 0, 0, 0;
  mManeuver = {"null", 0};
}

Ship::Ship(const sManeuver aManeuver, const std::string aType)
{
  double m = 0, mX = 0, mY = 0;
  double xG = 0, iZ = 0, jZ = 0;

  Eigen::Matrix3d matM;

  if(0 == setShipParams(aType))
    {
      m = mRho * mGeoParams.volume;
      mX = 0.5 * mRho * pow(mGeoParams.lPP, 2) * mGeoParams.d * mAddedMassParams.mpX;
      mY = 0.5 * mRho * pow(mGeoParams.lPP, 2) * mGeoParams.d * mAddedMassParams.mpY;

      xG = mGeoParams.xG;
      iZ = m * pow((0.25 * mGeoParams.lPP), 2);
      jZ = 0.5 * mRho * pow(mGeoParams.lPP, 4) * mGeoParams.d * mAddedMassParams.jpZ;

      matM << m+mX, 0, 0,
	0, m+mY, xG*m,
	0, xG*m, iZ+(m*pow(xG, 2))+jZ;

      //Fill class members
      mM = m;
      mMX = mX;
      mMY = mY;
      mMatM = matM;
      mInvMatM = matM.inverse();
      mMu = mMu0;
      mEta << 0, 0, 0;

      mManeuver = aManeuver;
      mWind.InitVectC(mGeoParams, mShipWindParams);
      mWind.InitVectABC();
      
      //std::cout << "Mat M inverse : " << matM.inverse() << std::endl; 
    }
  else
    {
      Ship();
    }
}

void Ship::setEta(const Eigen::Vector3d& aEta){mEta << aEta;}

void Ship::setMu(const Eigen::Vector3d& aMu){mMu << aMu;}

int Ship::setShipParams(const std::string aType)
{
  int ret = 0;

  /*TODO : Get datas from parameters file*/
  if("kvlcc2" == aType)
    {
      mRho = 1025;
      mMu0 << 15.5 * 0.514, 0, 0;
      mGeoParams = {320, 58, 20.8, 312622, 11.1, 0.81};
      mHullDerParams = {0.022,-0.04, 0.002, 0.011, 0.771, -0.315, 0.083, -1.607, 0.379, -0.391, 0.008, -0.137, -0.049, -0.03, -0.294, 0.055, -0.013};
      mAddedMassParams = {0.022, 0.223, 0.011};
      mPropellerParams = {9.86, 0.22, -0.48, 0.35, 0.293, -0.275, -0.139, 1.53};
      mRudderParams = {15.8, 112.5, -0.5, 0.312, 0.387, -0.464, 1.09, 0.5, -0.71, 1.827, {0.395, 0.64}, 0.0407};
      mShipWindParams = {4910, 1624, 750, 375, 160, 0, 1.225, 0 * 15.5 * 0.514, 90};  
    }
  else
    {
      ret = -1;
    }
  
  return ret;
}

int Ship::Controller(const double dt)
{
  double delta = 0;
  
  if("turning_cycle" == mManeuver.type)
    {
      delta = mManeuver.params;
      //std::cout << "turning_cycle : delta :" << delta << std::endl; 
    }
  else if("zig_zag" == mManeuver.type)
    {
      if(abs(mEta[2]) > abs(mManeuver.params) && (toolsSign(mEta[2]) == (toolsSign(mManeuver.params))))
	{
	  mManeuver.params = -mManeuver.params;
	}
      
      delta = mManeuver.params;
      //std::cout << "zig_zag : delta :" << delta << std::endl; 
    }
  else return -1;

  getRudd().SetDelta(delta, dt, getRudderParams());

  return 0;
}

Propeller& Ship::getProp(void){return mProp;}
Hull& Ship::getHull(void){return mHull;}
Rudder& Ship::getRudd(void){return mRudd;}
Wind& Ship::getWind(void){return mWind;}

sHullDerParams& Ship::getHullDerParams(void){return mHullDerParams;}
sAddedMassParams& Ship::getAddedMassParams(void){return mAddedMassParams;}
sPropellerParams& Ship::getPropellerParams(void){return mPropellerParams;}
sRudderParams& Ship::getRudderParams(void){return mRudderParams;}
sShipWindParams& Ship::getShipWindParams(void){return mShipWindParams;}
sGeoParams& Ship::getGeoParams(void){return mGeoParams;}

double Ship::getRho(void){return mRho;}
double Ship::getM(void){return mM;}
double Ship::getMX(void){return mMX;}
double Ship::getMY(void){return mMY;}
Eigen::Vector3d& Ship::getMu0(void){return mMu0;}
Eigen::Vector3d& Ship::getMu(void){return mMu;}
Eigen::Vector3d& Ship::getEta(void){return mEta;}
Eigen::Matrix3d& Ship::getInvMatM(void){return mInvMatM;}

