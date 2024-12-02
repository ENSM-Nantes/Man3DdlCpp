#include <Eigen/Dense>
#include <cmath>
#include "tools.h"
#include "Wind.h"

Wind::Wind()
{
  mT << 0, 0, 0;
}

Wind::Wind(const sGeoParams& aGeo, const sShipWindParams& aWind)
{
  mT << 0, 0, 0;
  InitVectC(aGeo, aWind);
  InitVectABC();
}
  
void Wind::ComputeT(const Eigen::Vector3d& aMu, const Eigen::Vector3d& aEta, const sGeoParams& aGeo, const sShipWindParams& aWind)
{ 
  double tws = 0, twa = 0, bs = 0, ur = 0;
  double gammar = 0, cx = 0, cy = 0, cn = 0;
  double xw = 0, yw = 0, nw = 0;
  int sign = 0;
  
  tws = aWind.velWind;
  twa = (aWind.psiWind * M_PI/180) - aEta[2];
  bs = pow((pow(aMu[0], 2) + pow(aMu[1], 2)), 0.5);
  ur = pow((pow(bs, 2) + pow(tws, 2) + 2*bs*tws*cos(twa)), 0.5);

  if(ur != 0) gammar = atan((tws * sin(twa))/(tws*cos(twa)+bs));
  else gammar = 0;

  sign = toolsSign(gammar);

  mVectA.resize(mFnSplineANbr);
  mVectB.resize(mFnSplineBNbr);
  mVectC.resize(mFnSplineCNbr);
  
  for(int r=0;r<mFnSplineANbr;r++) mVectA(r) = mFnSplineVectA[r](sign*gammar).coeff(0);

  for(int r=0;r<mFnSplineBNbr;r++) mVectB(r) = mFnSplineVectB[r](sign*gammar).coeff(0);
  
  for(int r=0;r<mFnSplineCNbr;r++) mVectC(r) = mFnSplineVectC[r](sign*gammar).coeff(0);
  
  cx = -mVectA.dot(mVectCX);
  cy = (-mVectB.dot(mVectCY))*sign;
  cn = (-mVectC.dot(mVectCN))*sign;

  xw = 0.5 * aWind.rho * aWind.aT * cx;
  yw = 0.5 * aWind.rho * aWind.aL * cy;
  nw = 0.5 * aWind.rho * aWind.aL * cn * aGeo.lPP;
  
  mT << xw * pow(ur, 2), yw * pow(ur, 2), nw * pow(ur, 2);

  //std::cout << "****Wind mT :" << mT << std::endl; 
  return;
}

void Wind::InitVectC(const sGeoParams& aGeo, const sShipWindParams& aWind)
{
  double b = 0, lpp = 0, ass = 0, al = 0, at = 0;
  double s = 0, c = 0, m = 0;

  b = aGeo.b;
  lpp = aGeo.lPP;
  al = aWind.aSS;
  ass = aWind.aL;
  at = aWind.aT;
  s = aWind.S;
  c = aWind.C;
  m = aWind.M;

  mVectCX.resize(BUFFER_VECTOR_SIZE);
  mVectCY.resize(BUFFER_VECTOR_SIZE);
  mVectCN.resize(BUFFER_VECTOR_SIZE-1);
  
  mVectCX << 1, 2*al/pow(lpp, 2), 2*at/pow(b, 2), lpp/b, s/lpp, c/lpp, m;
  mVectCY << 1, 2*al/pow(lpp, 2), 2*at/pow(b, 2), lpp/b, s/lpp, c/lpp, ass/al;
  mVectCN << 1, 2*al/pow(lpp, 2), 2*at/pow(b, 2), lpp/b, s/lpp, c/lpp;
  
  return;
}

int Wind::InitVectABC(void)
{
  /*TODO : Load data from a file --> /!\ monitor execution time */
  Eigen::MatrixXd matA(19,7);
  Eigen::MatrixXd matB(19,7);
  Eigen::MatrixXd matC(19,6);

  Eigen::MatrixXd matAT(7,19);
  Eigen::MatrixXd matBT(7,19);
  Eigen::MatrixXd matCT(6,19);
  
  Eigen::RowVectorXd gammar(19);
  Eigen::RowVectorXd signal(19);

  matA << 2.152, -5.00, 0.243, -0.164, 0, 0, 0,
    1.714, -3.33, 0.145, -0.121, 0, 0, 0,
    1.818, -3.97, 0.211, -0.143, 0, 0, 0.033,
    1.965, -4.81, 0.243, -0.154, 0, 0, 0.041,
    2.333, -5.99, 0.247, -0.190, 0, 0, 0.042,
    1.726, -6.54, 0.189, -0.173, 0.348, 0, 0.048,
    0.913, -4.68, 0, -0.104, 0.482, 0, 0.052,
    0.457, -2.88, 0, -0.068, 0.346, 0, 0.053,
    0.341, -0.91, 0, -0.031, 0, 0, 0.032,
    0.355, 0, 0, 0, -0.247, 0, 0.018,
    0.601, 0, 0, 0, -0.372, 0, -0.02,
    0.651, 1.29, 0, 0, -0.582, 0, -0.031,
    0.564, 2.54, 0, 0, -0.748, 0, -0.024,
    -0.142, 3.58, 0, 0.047, -0.700, 0, -0.028,
    -0.677, 3.64, 0, 0.069, -0.529, 0, -0.032,
    -0.723, 3.14, 0, 0.064, -0.475, 0, -0.032,
    -2.148, 2.56, 0, 0.081, 0, 1.27, -0.027,
    -2.707, 3.97, -0.175, 0.126, 0, 1.81, 0,
    -2.529, 3.76, -0.174, 0.128, 0, 1.55, 0; 

  matAT = matA.transpose();
  
  matB << 0, 0, 0, 0, 0, 0, 0,
    0.096, 0.22, 0, 0, 0, 0, 0,
    0.176, 0.71, 0, 0, 0, 0, 0,
    0.225, 1.38, 0, 0.023, 0, -0.29, 0,
    0.329, 1.82, 0, 0.043, 0, -0.59, 0,
    1.164, 1.26, 0.121, 0, -0.242, -0.95, 0,
    1.163, 0.96, 0.101, 0, -0.177, -0.88, 0,
    0.916, 0.53, 0.069, 0, 0, -0.65, 0,
    0.844, 0.55, 0.082, 0, 0, -0.54, 0,
    0.889, 0, 0.138, 0, 0, -0.66, 0,
    0.799, 0, 0.155, 0, 0, -0.55, 0,
    0.797, 0, 0.151, 0, 0, -0.55, 0,
    0.996, 0, 0.184, 0, -0.212, -0.66, 0.34,
    1.014, 0, 0.191, 0, -0.28, -0.69, 0.44,
    0.784, 0, 0.166, 0, -0.209, -0.53, 0.38,
    0.536, 0, 0.176, -0.029, -0.163, 0, 0.27,
    0.251, 0, 0.106, -0.022, 0, 0, 0,
    0.125, 0, 0.046, -0.012, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;
  
  matBT = matB.transpose();
  
  matC << 0, 0, 0, 0, 0, 0,
    0.0596, 0.061, 0, 0, 0, -0.074,
    0.1106, 0.204, 0, 0, 0, -0.17,
    0.2258, 0.245, 0, 0, 0, -0.38,
    0.2017, 0.457, 0, 0.0067, 0, -0.472,
    0.1759, 0.573, 0, 0.0118, 0, -0.523,
    0.1925, 0.48, 0, 0.0115, 0, -0.546,
    0.2133, 0.315, 0, 0.0081, 0, -0.526,
    0.1827, 0.254, 0, 0.0053, 0, -0.443,
    0.2627, 0, 0, 0, 0, -0.508,
    0.2102, 0, -0.0195, 0, 0.0335, -0.492,
    0.1567, 0, -0.0258, 0, 0.0497, -0.457,
    0.0801, 0, -0.0311, 0, 0.074, -0.396,
    -0.0189, 0, -0.0488, 0.0101, 0.1128, -0.42,
    0.0256, 0, -0.0422, 0.01, 0.0889, -0.463,
    0.0552, 0, -0.0381, 0.0109, 0.0689, -0.476,
    0.0881, 0, -0.0306, 0.0091, 0.0366, -0.415,
    0.0851, 0, -0.0122, 0, 0.0025, -0.22,
    0, 0, 0, 0, 0, 0;

  matCT = matC.transpose();
  
  /*Constant /-/ 0° -> 180° /-/ 10° step*/
  gammar << 0, 0.17453293, 0.34906585, 0.52359878, 0.6981317, 0.87266463,
    1.04719755, 1.22173048, 1.3962634, 1.57079633, 1.74532925, 1.91986218,
    2.0943951,  2.26892803, 2.44346095, 2.61799388, 2.7925268,  2.96705973,
    3.14159265;

  if(matAT.rows() > SPLINE_FUNCTION_MAX)
    {
      return -1;
    }
  else
    {
      /*Linear Interpolation*/
      for(int r=0; r<matAT.rows();r++)
	{
	  signal << matAT.row(r);
	  auto fit = SplineFitting1D::Interpolate(signal, 1, gammar);
	  Spline1D spline(fit);
	  mFnSplineVectA[r] = spline;
	}
      mFnSplineANbr = matAT.rows();
    }

  if(matBT.rows() > SPLINE_FUNCTION_MAX)
    {
      return -1;
    }
  else
    {
      /*Linear Interpolation*/
      for(int r=0; r<matBT.rows();r++)
	{
	  signal << matBT.row(r);
	  auto fit = SplineFitting1D::Interpolate(signal, 1, gammar);
	  Spline1D spline(fit);
	  mFnSplineVectB[r] = spline;
	}
      mFnSplineBNbr = matBT.rows();
    }

  if(matCT.rows() > SPLINE_FUNCTION_MAX)
    {
      return -1;
    }
  else
    {
      /*Linear Interpolation*/
      for(int r=0; r<matCT.rows();r++)
	{
	  signal << matCT.row(r);
	  auto fit = SplineFitting1D::Interpolate(signal, 1, gammar);
	  Spline1D spline(fit);
	  mFnSplineVectC[r] = spline;
	}
      mFnSplineCNbr = matCT.rows();
    }

  return 0;
}

Eigen::Vector3d& Wind::getWindT(void){return mT;}
