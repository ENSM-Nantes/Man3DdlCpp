#include <iostream>
#include <fstream> 
#include <time.h>      
#include "Ship.h"
#include "Solver.h"
#include "tools.h"

#define OCTAVE_SCRIPT(A)			\
  if (system("octave " A) != 0) {		\
    printf("Octave script [%s] failed !", A);	\
    exit(-1);					\
  }

void savePlot(unsigned char nPlot, Eigen::VectorXd &x, Eigen::VectorXd &y, double aPrm1, double aPrm2)
{
  long i = 0, j = 0, di = 0, dj = 0;

  if(0 == nPlot)
    {
      std::ofstream plotFileTraj("./plot/savePlotTraj.csv");
      i = 1;
      j = 0;
      di = 3;
      dj = 3;

      for (i,j ; i < y.size(); i+=di,j+=dj)
	plotFileTraj << x(i)/aPrm1 << "," << y(j)/aPrm1 << std::endl;

      plotFileTraj.close();
    }
  else if(1 == nPlot)
    {
      std::ofstream plotFileU("./plot/savePlotU.csv");
      i = 0;
      j = 0;
      di = 1;
      dj = 3;

      for (i,j ; i < x.size(); i+=di,j+=dj)
	plotFileU << x(i)*aPrm2/aPrm1 << "," << pow((pow(y(j), 2) + pow(y(j+1), 2)), 0.5) / aPrm2 << std::endl;

      
      plotFileU.close();
    }
  else if(2 == nPlot)
    {
      std::ofstream plotFileR("./plot/savePlotR.csv");
      i = 0;
      j = 2;
      di = 1;
      dj = 3;

      for (i,j ; i < x.size(); i+=di,j+=dj)
	plotFileR << x(i)*aPrm2/aPrm1 << "," << y(j)*aPrm1/aPrm2 << std::endl;
      
      plotFileR.close();
    }
  else if(3 == nPlot)
    {
      std::ofstream plotFileDrift("./plot/savePlotDrift.csv");
      i = 0;
      j = 0;
      di = 1;
      dj = 3;

      for (i,j ; i < x.size(); i+=di,j+=dj)
	plotFileDrift << x(i)*aPrm2/aPrm1 << "," << toolsRadToDeg(atan(-y(j+1)/y(j))) << std::endl;
      
      plotFileDrift.close();
    }
  else if(4 == nPlot)
    {
      std::ofstream plotFileRudd("./plot/savePlotRudd.csv");
      i = 0;
      j = 0;
      di = 1;
      dj = 1;

      for (i,j ; i < x.size(); i+=di,j+=dj)
	plotFileRudd << x(i)*aPrm2/aPrm1 << "," << toolsRadToDeg(y(j)) << std::endl;
      
      plotFileRudd.close();
    }
  else if(5 == nPlot)
    {
      std::ofstream plotFileHeading("./plot/savePlotHeading.csv");
      i = 0;
      j = 2;
      di = 1;
      dj = 3;

      for (i,j ; i < x.size(); i+=di,j+=dj)
	plotFileHeading << x(i)*aPrm2/aPrm1 << "," << toolsRadToDeg(y(j)) << std::endl;
      
      plotFileHeading.close();
    }
  else return;
  
}

int main(int argc, char* argv[])
{
  double time_spent = 0.0;
  clock_t begin = clock();

  std::string shipType = argv[1];
  std::string manStr = argv[2];

  const sManeuver maneuver = {manStr, toolsDegToRad(atoi(argv[3]))};
  Ship ship1(maneuver, shipType);
  sDynParams params = {0, 1500, 1, ship1};
  Solver solver1(params);

  /*Test Hull*/
  //Eigen::Vector3d mu(2, -0.8, 0.007);
  //ship1.getHull().ComputeT(mu, ship1.getRho(), ship1.getGeoParams(), ship1.getHullDerParams());

  /*Test Propeller*/
  //ship1.getProp().ComputeT(mu, ship1.getRho(), ship1.getGeoParams(), ship1.getPropellerParams());

  /*Test Propeller*/
  //ship1.getRudd().ComputeT(mu, ship1.getRho(), ship1.getGeoParams(), ship1.getRudderParams(), ship1.getPropellerParams());

  /*Test DiffEq*/
  //Eigen::VectorXd etaMu(6);
  //etaMu << 250, 680, 11, 2, -0.8, 0.007;
  //solver1.DiffEq(etaMu);

  /*Launch Solver*/
  solver1.SolveDyn();

  clock_t end = clock();
  time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

  std::cout << "time spent : " << time_spent << std::endl;

  savePlot(0, solver1.getOutEta(), solver1.getOutEta(), ship1.getGeoParams().lPP, 0);
  savePlot(1, solver1.getOutT(), solver1.getOutMu(), ship1.getGeoParams().lPP, ship1.getMu0()[0]);
  savePlot(2, solver1.getOutT(), solver1.getOutMu(), ship1.getGeoParams().lPP, ship1.getMu0()[0]);
  savePlot(3, solver1.getOutT(), solver1.getOutMu(), ship1.getGeoParams().lPP, ship1.getMu0()[0]);
  savePlot(4, solver1.getOutT(), solver1.getOutCtrl(), ship1.getGeoParams().lPP, ship1.getMu0()[0]);
  savePlot(5, solver1.getOutT(), solver1.getOutEta(), ship1.getGeoParams().lPP, ship1.getMu0()[0]);

  if("turning_cycle" == maneuver.type){ OCTAVE_SCRIPT("./plot/plot.m 2");}
  else if("zig_zag" == maneuver.type){ OCTAVE_SCRIPT("./plot/plot.m 1");}
  
  
  return 0;
}
