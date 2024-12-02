#ifndef PROPELLER_H
#define PROPELLER_H

#include <Eigen/Dense>
#include <vector>
#include "ShipParams.h"

class Propeller
{
public:
  
  Propeller();

  /*Process*/
  void ComputeT(const Eigen::Vector3d& aMu, double aRho, const sGeoParams& aGeo, const sPropellerParams& aProp);

  /*Getter*/
  Eigen::Vector3d& getPropellerT(void);
  
private: 

  Eigen::Vector3d mT;
  
};

#endif
