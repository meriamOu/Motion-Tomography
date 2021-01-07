#include <iostream>
#include <vector> 
#include <map>
#include <string>

using namespace std;
class CellStorage
{
public:
   std::map< std::pair<int, int>, vector<float>> storageMap;
    float xDim;
    float yDim;
    float xMin;
    float yMin;
    int xCellheight;
    int yCellheight;
public:  

    CellStorage( const float & xDim,  const float & yDim,    const float & xMin,   const float & yMin,    const int & xCellheight,   const int & yCellheight);
    vector<float> getFlow(const float & posX, const float & posY ) {
        if (posX<xMin || posX>xMin+ xCellheight*xDim || posY<yMin || posY>xMin+ yCellheight*yDim){
            throw "invalid CellStorage range!";
        }

        std::pair <int, int > position;
        position.first=(int)(posX - xMin)/xCellheight;
        position.second=(int)(posY - yMin)/yCellheight;
        return storageMap[position];

    }
    void setFlow(const float & posX, const float & posY, vector<float> Flow){
        if (posX<xMin || posX>xMin+ xCellheight*xDim || posY<yMin || posY>xMin+ yCellheight*yDim){
            throw "invalid CellStorage range!";
        }

        std::pair <int, int > position;
        position.first=(int)(posX - xMin)/xCellheight;
        position.second=(int)(posY - yMin)/yCellheight;
        storageMap[position]=Flow;

    }
};

CellStorage::CellStorage( const float & xDim,  const float & yDim,    const float & xMin,   const float & yMin,    const int & xCellheight,   const int & yCellheight){
    CellStorage::xCellheight=xCellheight;
    CellStorage::xDim=xDim;
    CellStorage::xMin=xMin;
    CellStorage::yCellheight=yCellheight;
    CellStorage::yDim=yDim;
    CellStorage::yMin=yMin;
}
 /* ask why 
vector<float> CellStorage::getFlow(const float & posX, const float & posY ){

}
 */ 

int main()
{

 CellStorage  FlowMap( 10,  10,    -5,   -5,    2,   2) ;
 FlowMap.setFlow(0,0,{-1,2});
 cout << FlowMap.getFlow(0,0)[0];
}

 



