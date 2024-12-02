#include <cmath>
#include "Solver.h"


Solver::Solver(const sDynParams& aDynParams)
{
  mt = aDynParams.t0;
  mtMax = aDynParams.tMax;
  mdT = aDynParams.dT;
  mShip = aDynParams.ship;
  mT << 0, 0, 0;
}

Eigen::VectorXd Solver::DiffEq(const Eigen::VectorXd& aVectEtaMu)
{
  const Eigen::Vector3d vEta(aVectEtaMu[0],aVectEtaMu[1],aVectEtaMu[2]);
  const Eigen::Vector3d vMu(aVectEtaMu[3],aVectEtaMu[4],aVectEtaMu[5]);
  Eigen::Matrix3d matRotPsi;
  Eigen::Matrix3d matC;
  Eigen::Vector3d vEtaP;
  Eigen::Vector3d vMuP;
  Eigen::VectorXd catVect;

  double xG = 0; 

  matRotPsi << cos(vEta[2]), -sin(vEta[2]) , 0,
    sin(vEta[2]), cos(vEta[2]), 0,
    0, 0 , 1;
  
  vEtaP = matRotPsi * vMu;

  xG = mShip.getGeoParams().xG;
  matC << 0, -(mShip.getM()+mShip.getMY())*vMu[2], -xG*mShip.getM()*vMu[2],
    (mShip.getM()+mShip.getMX())*vMu[2], 0, 0, xG*mShip.getM()*vMu[2], 0, 0;
  
  mShip.getHull().ComputeT(vMu, mShip.getRho(), mShip.getGeoParams(), mShip.getHullDerParams());
  mShip.getProp().ComputeT(vMu, mShip.getRho(), mShip.getGeoParams(), mShip.getPropellerParams());
  mShip.getRudd().ComputeT(vMu, mShip.getRho(), mShip.getGeoParams(), mShip.getRudderParams(), mShip.getPropellerParams());

  mT << mShip.getHull().getHullT() + mShip.getProp().getPropellerT() + mShip.getRudd().getRudderT();
  
  mShip.getWind().ComputeT(vMu, vEta, mShip.getGeoParams(), mShip.getShipWindParams());

  mT += mShip.getWind().getWindT();
  
  vMuP = mShip.getInvMatM() * (mT - (matC * vMu));
  
  catVect.resize(vEtaP.size() + vMuP.size());
  catVect << vEtaP, vMuP;
  
  return catVect;
}

void Solver::SolveDyn(void)
{
  Eigen::VectorXd eta;
  Eigen::VectorXd mu;
  Eigen::VectorXd t;
  Eigen::VectorXd control;
  Eigen::VectorXd tmp;
  unsigned short i = 0;
  
  while(mt < mtMax)
    {
      mShip.Controller(mdT);
      SolveRk4();

      tmp.resize(eta.size());
      tmp << eta;
      eta.resize(eta.size() + mShip.getEta().size());
      eta << tmp, mShip.getEta();

      tmp.resize(mu.size());
      tmp << mu;
      mu.resize(mu.size() + mShip.getMu().size());
      mu << tmp, mShip.getMu();

      tmp.resize(t.size());
      tmp << t;
      t.resize(t.size() + 1);
      t << tmp, mt;

      tmp.resize(control.size());
      tmp << control;
      control.resize(control.size() + 1);
      control << tmp, mShip.getRudd().getDelta();
    }

  mOutEta.resize(eta.size());
  mOutMu.resize(mu.size());
  mOutT.resize(t.size());
  mOutCtrl.resize(control.size());
  
  mOutEta << eta;
  mOutMu << mu;
  mOutT << t;
  mOutCtrl << control;
}

void Solver::SolveRk4(void)
{
  Eigen::VectorXd y(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd tmp(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd dy1(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd dy2(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd dy3(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd dy4(VECTOR_SIZE_DIFF_EQ);
  Eigen::VectorXd ySol(VECTOR_SIZE_DIFF_EQ);
  Eigen::Vector3d eta;
  Eigen::Vector3d mu;
  
  //std::cout << "****SolveRk4****" << std::endl;
  
  y << mShip.getEta(), mShip.getMu();

  //std::cout << "y : " << y << std::endl; 

  dy1 = DiffEq(y);
  tmp = y + (0.5 * mdT * dy1);

  dy2 = DiffEq(tmp);
  tmp = y + (0.5 * mdT * dy2);

  dy3 = DiffEq(tmp);
  tmp = y + mdT * dy3;

  dy4 = DiffEq(tmp);
  ySol = y + mdT * (dy1 + 2*dy2 + 2*dy3 + dy4)/6;

  mt = mt + mdT;

  eta << ySol[0], ySol[1], ySol[2];
  mu << ySol[3], ySol[4], ySol[5];

  //std::cout << "eta : " << eta << std::endl;
  //std::cout << "mu : " << mu << std::endl; 
  
  mShip.setEta(eta);
  mShip.setMu(mu);
}

Eigen::VectorXd& Solver::getOutEta(void){return mOutEta;}
Eigen::VectorXd& Solver::getOutMu(void){return mOutMu;}
Eigen::VectorXd& Solver::getOutT(void){return mOutT;}
Eigen::VectorXd& Solver::getOutCtrl(void){return mOutCtrl;}
