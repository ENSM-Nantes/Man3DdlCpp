#ifndef RUDDER_H
#define RUDDER_H

#include <Eigen/Dense>
#include <vector>
#include "ShipParams.h"

class Rudder
{
public:
  
  Rudder();

  /*Process*/
  void ComputeT(const Eigen::Vector3d& aMu, const double aRho, const sGeoParams& aGeo, const sRudderParams& aRudder, const sPropellerParams& aProp);

  /*Setter*/
  void SetDelta(const double aDelta, const double aDt, const sRudderParams& aRudder);

  /*Getter*/
  Eigen::Vector3d& getRudderT(void);
  double getDelta(void);
  
private: 
  
  Eigen::Vector3d mT;
  double mDelta;
};

#endif
