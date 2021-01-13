#include <iostream>

//#include <ChStream.h>

#include "CellStorage.h"
#include <Eigen/Dense>
#include <vector>

#include "gnuplot-iostream.h"
using namespace Eigen;

using namespace std;

struct vehicle
{
public:
  RowVector2d initialPosition;
  RowVector2d finalPosition;
  RowVector2d controlSpeed;
  double travelTime;
  RowVector2d MTerror;
  CellStorage cellTime;

public:
  vehicle() : cellTime(10, 10, -5, -5, 1, 1) {}
};

void convertFlowMapToVector(CellStorage &FlowMap, MatrixXd &FlowVect)
{
  int k = 0;
  RowVector2d FlowInCell;
  for (int j = 0; j < FlowMap.yDim; j++)
  {
    for (int i = 0; i < FlowMap.xDim; i++)
    {
      std::pair<int, int> position;
      position.first = i;
      position.second = j;

      FlowInCell = FlowMap.storageMap[position];
      FlowVect(k, 0) = FlowInCell(0, 0);
      FlowVect(k, 1) = FlowInCell(0, 1);

      k++;
    }
  }
}

void convertTimeMap(CellStorage &cellTime, MatrixXd &travelTimeAUV)
{
  int k = 0;
  RowVector2d travelTimeInCell;
  for (int j = 0; j < cellTime.yDim; j++)
  {
    for (int i = 0; i < cellTime.xDim; i++)
    {
      std::pair<int, int> position;
      position.first = i;
      position.second = j;

      travelTimeInCell = cellTime.storageMap[position];
      travelTimeAUV(k, 0) = travelTimeInCell(0, 0);
      ///reset the time in cell
      cellTime.storageMap[position] = 0 * travelTimeInCell;

      k++;
    }
  }
}
void convertVectorFlowToMap(CellStorage &FlowMap, MatrixXd &FlowVect)
{
  int k = 0;

  for (int j = 0; j < FlowMap.yDim; j++)
  {
    for (int i = 0; i < FlowMap.xDim; i++)
    {
      std::pair<int, int> position;
      position.first = i;
      position.second = j;

      FlowMap.storageMap[position] = FlowVect.row(k);

      k++;
    }
  }
}
RowVector2d simulator(double &dt, vehicle &AUV, CellStorage &FlowMap)
{
  int steps = (int)(AUV.travelTime / dt);
  RowVector2d currentPosition(1, 2);
  currentPosition = AUV.initialPosition;
  RowVector2d currentFlow;
  double auxTime;
  int i = 0;
  while (i < steps)
  {

    auxTime = AUV.cellTime.getTravelTime(currentPosition(0, 0), currentPosition(0, 1));
    AUV.cellTime.setTraveltime(currentPosition(0, 0), currentPosition(0, 1), auxTime + dt);
    currentFlow = FlowMap.getFlow(currentPosition(0, 0), currentPosition(0, 1));

    currentPosition = currentPosition + dt * (currentFlow + AUV.controlSpeed);

    i++;
  }
  AUV.MTerror = AUV.finalPosition - currentPosition;

  return currentPosition;
}

MatrixXd simulatorTrajectory(double &dt, vehicle &AUV, CellStorage &FlowMap)
{
  int steps = (int)(AUV.travelTime / dt);

  MatrixXd trajectory(steps, 2);
  RowVector2d currentPosition(1, 2);
  currentPosition = AUV.initialPosition;

  RowVector2d currentFlow;
  double auxTime;
  double time = 0;

  int i = 0;
  while (i < steps)
  {

    auxTime = AUV.cellTime.getTravelTime(currentPosition(0, 0), currentPosition(0, 1));
    AUV.cellTime.setTraveltime(currentPosition(0, 0), currentPosition(0, 1), auxTime + dt);
    currentFlow = FlowMap.getFlow(currentPosition(0, 0), currentPosition(0, 1));

    currentPosition = currentPosition + dt * (currentFlow + AUV.controlSpeed);

    trajectory(i, 0) = currentPosition(0, 0);
    trajectory(i, 1) = currentPosition(0, 1);

    i++;
  }

  return trajectory;
}

int main(int, char **)
{
  int numberIteration = 200;
  double omega = 0.5;
  int numberAUVs = 18;
  int xDim = 10;
  int xMin = -5;
  int xCellheight = 1;
  int yCellheight = 1;
  int yDim = 10;
  int yMin = -5;
  int mapSize = xDim * yDim;
  MatrixXd FlowVect(mapSize, 2);
  MatrixXd TrueFlowVect(mapSize, 2);
  MatrixXd FlowEstimationError(mapSize, 2);
  MatrixXd travelTimeAUV(mapSize, 1);

  MatrixXd travelTimeTotalAUV(mapSize, numberAUVs);

  CellStorage FlowMap(xDim, yDim, xMin, yMin, xCellheight, yCellheight);
  CellStorage TrueFlowMap(xDim, yDim, xMin, yMin, xCellheight, yCellheight);
  RowVector2d initalFlow;
  RowVector2d controlSpeed;
  initalFlow << 0, 0;
  FlowMap.initializeFlow(initalFlow);
  initalFlow << 0.15, 0;
  TrueFlowMap.initializeFlow(initalFlow);

  for (int j = 1; j < 11; j++)
  {
    for (int i = 1; i < 11; i++)
    {

      double Fx = -(j) / (2 * 3.14 * sqrt(i * i + j * j));
      double Fy = (i) / (2 * 3.14 * sqrt(i * i + j * j));
      initalFlow << Fx, Fy;

      std::pair<int, int> position;
      position.first = i;
      position.second = j;
      TrueFlowMap.storageMap[position] = initalFlow;
    }
  }

  convertFlowMapToVector(TrueFlowMap, TrueFlowVect);

  double dt = 0.1;
  MatrixXd MTerror(numberAUVs, 2);
  // Simulation Setup

  double dx, dy, posx, posy;
  vector<vehicle *> vehicleVec;
  //numberAUVs
  for (int i = 0; i < numberAUVs; i++)
  {

    vehicle *AUV = new vehicle();
    if (i < 9)
    {
      dx = 1;
      dy = 0;
      controlSpeed << 0, 0.3;
      posx = -5.5 + (i + 1) * dx;
      posy = -5;
    }
    else
    {
      dx = 0;
      dy = 1;
      controlSpeed << 0.3, 0;
      posx = -5;
      posy = -5.5 + (i - 7) * dy;
    }

    AUV->initialPosition << posx, posy;
    AUV->controlSpeed << controlSpeed;
    AUV->travelTime = 18.5;
    AUV->MTerror << 0, 0;
    AUV->finalPosition << 0, 0;

    AUV->finalPosition = simulator(dt, *AUV, TrueFlowMap);

    AUV->cellTime.initializeTraveltime(0);

    vehicleVec.push_back(AUV);
  }

  convertFlowMapToVector(FlowMap, FlowVect);
  RowVector2d predictedFinalPosition;
  int n = 0;
  //numberIteration
  while (n < numberIteration)
  {
    n++;
    // Forward Step

    for (int i = 0; i < numberAUVs; i++)
    {

      predictedFinalPosition = simulator(dt, *((vehicleVec[i])), FlowMap);

      MTerror.row(i) = (vehicleVec[i])->MTerror;

      convertTimeMap((vehicleVec[i])->cellTime, travelTimeAUV);

      travelTimeTotalAUV.col(i) = travelTimeAUV;
    }

    // Inverse Step
    double inversNorm = 1 / travelTimeTotalAUV.norm();
    FlowVect = FlowVect + 0.5 * (inversNorm * inversNorm) * travelTimeTotalAUV * MTerror;

    convertVectorFlowToMap(FlowMap, FlowVect);

    FlowEstimationError = TrueFlowVect - FlowVect;
  }

  Gnuplot gp;

  std::vector<boost::tuple<double, double, double, double>> pts_A;
  std::vector<boost::tuple<double, double, double, double>> flowField;
  std::vector<boost::tuple<double, double, double, double>> trueFlowField;

  int k = 0;
  for (int j = 1; j < 11; j++)
  {
    for (int i = 1; i < 11; i++)
    {

      double Fx = FlowVect(k);
      double Fy = FlowVect(100 + k);
      flowField.push_back(boost::make_tuple(
          -5.5 + i,
          -5.5 + j,
          Fx,
          Fy));
      k++;
    }
  }

  k = 0;
  for (int j = 1; j < 11; j++)
  {
    for (int i = 1; i < 11; i++)
    {

      double Fx = TrueFlowVect(k);
      double Fy = TrueFlowVect(100 + k);

      trueFlowField.push_back(boost::make_tuple(
          -5.5 + i,
          -5.5 + j,
          Fx,
          Fy));
      k++;
    }
  }

  gp << "set xrange [-5:5]\nset yrange [-5:5]\n";
  // '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
  gp << "plot '-' with vectors title 'Estimated Flow Field', '-' with vectors title 'True Flow Field'\n";
  //	gp.send1d(pts_A);
  gp.send1d(flowField);
  gp.send1d(trueFlowField);
}
