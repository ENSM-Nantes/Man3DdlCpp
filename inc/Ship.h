#ifndef SHIP_H
#define SHIP_H

#include <iostream>
#include <Eigen/Dense>
#include "ShipParams.h"
#include "Propeller.h"
#include "Hull.h"
#include "Rudder.h"
#include "Wind.h"

typedef struct{  
  std::string type;
  double params;
}sManeuver;

class Ship
{
public:

  /*Constructor*/
  Ship();
  Ship(const sManeuver aManeuver, const std::string aType);

  /*Process*/
  int Controller(const double dt);
  
  /*Setter*/
  int setShipParams(const std::string aType);
  void setEta(const Eigen::Vector3d& aEta);
  void setMu(const Eigen::Vector3d& aMu);
  
  /*Getter*/
  //Initial parameters
  double getM(void);
  double getMY(void);
  double getMX(void);
  double getRho(void);
  Eigen::Vector3d& getMu0(void);
  Eigen::Matrix3d& getInvMatM(void);
  sGeoParams& getGeoParams(void);
  sShipWindParams& getShipWindParams(void);
  sHullDerParams& getHullDerParams(void);
  sAddedMassParams& getAddedMassParams(void);
  sPropellerParams& getPropellerParams(void);
  sRudderParams& getRudderParams(void);
  //Dynamic parameters
  Eigen::Vector3d& getEta(void);
  Eigen::Vector3d& getMu(void);
  //Boat parts
  Propeller& getProp(void);
  Hull& getHull(void);
  Rudder& getRudd(void);
  Wind& getWind(void);

private:
  //Initial params
  double mM;
  double mMX;
  double mMY;
  double mRho;
  Eigen::Matrix3d mMatM;  
  Eigen::Matrix3d mInvMatM;
  Eigen::Vector3d mMu0;
  sManeuver mManeuver;
  sGeoParams mGeoParams;
  sHullDerParams mHullDerParams;
  sAddedMassParams mAddedMassParams;
  sPropellerParams mPropellerParams;
  sRudderParams mRudderParams;
  sShipWindParams mShipWindParams;
  //Dynamic params
  Eigen::Vector3d mMu;
  Eigen::Vector3d mEta;
  //Boat parts
  Propeller mProp;
  Hull mHull;
  Rudder mRudd;
  Wind mWind;
};

#endif
