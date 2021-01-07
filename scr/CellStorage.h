#include <iostream>
#include <vector> 
#include <map>
#include <string>
#include <Eigen/Dense>
using namespace Eigen; 
using namespace std;
class CellStorage
{
public:
   std::map< std::pair<int, int>, Eigen::RowVector2d > storageMap;
    double xDim;
    double yDim;
    double xMin;
    double yMin;
    int xCellheight;
    int yCellheight;
public:  

    CellStorage( const double & xDim,  const double & yDim,    const double & xMin,   const double & yMin,    const int & xCellheight,   const int & yCellheight);
    RowVector2d getFlow(const double & posX, const double & posY ) ;
    double getTravelTime(const double & posX, const double & posY ) ;
    void setFlow(const double & posX, const double & posY, Eigen::RowVector2d Flow) ;
    void setTraveltime(const double & posX, const double & posY, const double travelTime);
    void initializeFlow( const Eigen::RowVector2d &initialFlow);
    void initializeTraveltime(const double &travelTime);
};

 
 



