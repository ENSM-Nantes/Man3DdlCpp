#ifndef WIND_H
#define WIND_H

#include <Eigen/Dense>
#include <vector>
#include "ShipParams.h"
#include <unsupported/Eigen/Splines>

typedef Eigen::Spline<double, 1> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

#define SPLINE_FUNCTION_MAX (10)
#define BUFFER_VECTOR_SIZE (7)

class Wind
{
public:

  /*Constructor*/
  Wind();
  Wind(const sGeoParams& aGeo, const sShipWindParams& aWind);

  /*Process*/
  int InitVectABC(void);
  void InitVectC(const sGeoParams& aGeo, const sShipWindParams& aWind);
  void ComputeT(const Eigen::Vector3d& aMu, const Eigen::Vector3d& aEta, const sGeoParams& aGeo, const sShipWindParams& aWind);

  /*Getter*/
  Eigen::Vector3d& getWindT(void);

private: 
  
  Eigen::Vector3d mT;
  unsigned char mFnSplineANbr;
  unsigned char mFnSplineBNbr;
  unsigned char mFnSplineCNbr;
  Spline1D mFnSplineVectA[SPLINE_FUNCTION_MAX];
  Spline1D mFnSplineVectB[SPLINE_FUNCTION_MAX];
  Spline1D mFnSplineVectC[SPLINE_FUNCTION_MAX];
  Eigen::VectorXd mVectA;
  Eigen::VectorXd mVectB;
  Eigen::VectorXd mVectC;
  Eigen::VectorXd mVectCX;
  Eigen::VectorXd mVectCY;
  Eigen::VectorXd mVectCN;
};

#endif
