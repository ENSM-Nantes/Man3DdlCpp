#ifndef HULL_H
#define HULL_H

#include <Eigen/Dense>
#include <vector>
#include "ShipParams.h"

class Hull
{
public:
  
  Hull();

  /*Process*/
  void ComputeT(const Eigen::Vector3d& aMu, const double aRho, const sGeoParams& aGeo, const sHullDerParams& aHull);

  /*Getter*/
  Eigen::Vector3d& getHullT(void);
  
private: 
  
  Eigen::Vector3d mT;
};

#endif
