#include <iostream>
#include <vector> 
#include <map>
#include <string>
#include "CellStorage.h"
#include <Eigen/Dense>
using namespace Eigen; 
using namespace std;

 
CellStorage::CellStorage( const double & xDim,  const double & yDim,    const double & xMin,   const double & yMin,    const int & xCellheight,   const int & yCellheight){
  
   this->xCellheight=xCellheight;
     this->xDim=xDim;
     this->xMin=xMin;
     this->yCellheight=yCellheight;
     this->yDim=yDim;
     this->yMin=yMin;
   
}
 
 RowVector2d CellStorage::getFlow(const double & posX, const double & posY ) {
        if (posX< this->xMin  || posX>this->xMin+ xCellheight*this->xDim || posY<this->yMin || posY>xMin+ yCellheight*this->yDim){
           
            throw "invalid CellStorage range!";
        }

        std::pair <int, int > position;
        
        position.first=(int)(posX - this->xMin)/this->xCellheight;
        position.second=(int)(posY - this->yMin)/this->yCellheight;
        RowVector2d flow =this->storageMap[position];
  

        return this->storageMap[position];

    }
    void CellStorage::setFlow(const double & posX, const double & posY, RowVector2d Flow){
       
        if (posX<this->xMin || posX>this->xMin+ this->xCellheight*this->xDim || posY<this->yMin || posY>this->xMin+ this->yCellheight*this->yDim){
            throw "invalid CellStorage range!";
        }
 
        std::pair <int, int > position;
        position.first=(int)(posX - this->xMin)/this->xCellheight;
        position.second=(int)(posY - this->yMin)/this->yCellheight;
        this->storageMap[position]=Flow;

    }
 

    double CellStorage::getTravelTime(const double & posX, const double & posY ) {
             if (posX< this->xMin  || posX>this->xMin+ xCellheight*this->xDim || posY<this->yMin || posY>xMin+ yCellheight*this->yDim){
           
            throw "invalid CellStorage range!";
        }

        std::pair <int, int > position;
        
        position.first=(int) ((posX - this->xMin)/this->xCellheight);

        position.second=(int)((posY - this->yMin)/this->yCellheight);
       // cout << posX<<"posX"<<((posX - this->xMin)/this->xCellheight)<<"posY"<<((posY - this->yMin)/this->yCellheight)<<"posY" <<posY<<endl;
        RowVector2d time =this->storageMap[position];
  

        return (this->storageMap[position])(0,1);
    }
    void CellStorage::setTraveltime(const double & posX, const double & posY, const double travelTime){
        
        if (posX<this->xMin || posX>this->xMin+ this->xCellheight*this->xDim || posY<this->yMin || posY>this->xMin+ this->yCellheight*this->yDim){
            throw "invalid CellStorage range!";
        }
        RowVector2d traveltimeVec ;
        traveltimeVec << travelTime, travelTime;
        std::pair <int, int > position;
        position.first=(int)(posX - this->xMin)/this->xCellheight;
        position.second=(int)(posY - this->yMin)/this->yCellheight;
        this->storageMap[position]=traveltimeVec;

    }

    void CellStorage::initializeFlow(const RowVector2d &initialFlow){
 
        for (int j=0;j<this->yDim;j++){
       
            
            for (int i=0;i<this->xDim;i++){
                std::pair <int, int > position;
                position.first=i;
                position.second=j;
                this->storageMap[position]=initialFlow;
  
              

                 }
                
                }
    }
    void CellStorage::initializeTraveltime(const double &travelTime){
        RowVector2d traveltimeVec ;
        traveltimeVec << travelTime, travelTime;

        for (int j=0;j<this->yDim;j++){
 
            
            for (int i=0;i<this->xDim;i++){
                std::pair <int, int > position;
                position.first=i;
                position.second=j;
                this->storageMap[position]=traveltimeVec;
        
                
               
                


            }
        
        }
    }




 

 



