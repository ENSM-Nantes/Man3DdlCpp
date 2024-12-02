#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <Eigen/Dense>
#include "Ship.h"

#define VECTOR_SIZE_DIFF_EQ (6)

typedef struct{

  double t0;
  double tMax;
  double dT;
  Ship ship;

}sDynParams;

class Solver
{
public:

  /*Constructor*/
  Solver(const sDynParams& aDynParams);

  /*Process*/
  Eigen::VectorXd DiffEq(const Eigen::VectorXd& aVectEtaMu);
  void SolveDyn(void);

  /*Getter*/
  Eigen::VectorXd& getOutEta(void);
  Eigen::VectorXd& getOutMu(void);
  Eigen::VectorXd& getOutT(void);
  Eigen::VectorXd& getOutCtrl(void);
  
private:

  void SolveRk4(void);
  double mt;
  double mtMax;
  double mdT;
  Ship mShip;
  Eigen::Vector3d mT;
  Eigen::VectorXd mOutEta;
  Eigen::VectorXd mOutMu;
  Eigen::VectorXd mOutT;
  Eigen::VectorXd mOutCtrl;
};

#endif
