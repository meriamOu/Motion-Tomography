#include <iostream>

//#include <ChStream.h>

#include "CellStorage.h"
#include <Eigen/Dense>
#include <vector>

 
 
#include "gnuplot-iostream.h"
using namespace Eigen; 


using namespace std;
 /*
int main() {
  std::vector<std::pair<double,double>> data;
  data.emplace_back(-2,-0.8);
  data.emplace_back(-1,-0.4);
  data.emplace_back(0,-0);
  data.emplace_back(1,0.4);
  data.emplace_back(1,0.8);
  Gnuplot gp;
  gp << "plot [-5:5] sin(x) tit 'sin(x)', '-' tit 'data'\n";
  gp.send1d(data);
  return 0;

  return 0;
}

*/


void  multiSum(vector<double> a, vector<double>b, double factor,  vector<double> &result){
int i=0;
for (int i=0;i<a.size();i++){
result[i]= factor*(a[i]+b[i])+result[i];
i++;
}}

struct vehicle{
     public: 
RowVector2d initialPosition    ;
RowVector2d  finalPosition  ;
RowVector2d  controlSpeed  ;
double travelTime;
RowVector2d MTerror  ;
CellStorage cellTime;
 public: 
// parameter lists : https://en.cppreference.com/w/cpp/language/constructor
 vehicle() : cellTime(10,  10,    -5,   -5,    1,   1 )   {  }
};

void convertFlowMapToVector( CellStorage  & FlowMap, MatrixXd & FlowVect ){
int k=0;
RowVector2d FlowInCell; 
for (int j=0;j<FlowMap.yDim;j++ ){
    for (int i=0;i<FlowMap.xDim;i++){
        std::pair <int, int > position;
                position.first=i;
                position.second=j;
                
                FlowInCell=FlowMap.storageMap[position];
                FlowVect(k,0)= FlowInCell(0,0);
                FlowVect(k,1)= FlowInCell(0,1);
        
        k++;

    }
}}

 void convertTimeMap(  CellStorage  &cellTime, MatrixXd & travelTimeAUV ){
int k=0;
RowVector2d travelTimeInCell;
for (int j=0;j<cellTime.yDim;j++ ){
    for (int i=0;i<cellTime.xDim;i++){
        std::pair <int, int > position;
                position.first=i;
                position.second=j;
                 
                travelTimeInCell=cellTime.storageMap[position];
                travelTimeAUV(k,0)= travelTimeInCell(0,0);
                ///reset the time in cell 
                cellTime.storageMap[position]=0*travelTimeInCell;
                
        
        k++;

    }
}

}
void convertVectorFlowToMap( CellStorage  & FlowMap, MatrixXd & FlowVect  ){
int k=0;
 
for (int j=0;j<FlowMap.yDim;j++ ){
    for (int i=0;i<FlowMap.xDim;i++){
        std::pair <int, int > position;
                position.first=i;
                position.second=j;
            
                FlowMap.storageMap[position] =FlowVect.row(k) ;
 
        
        k++;

    }
}


}  
RowVector2d simulator(  double &dt, vehicle  &AUV , CellStorage  &FlowMap ){
    int steps= (int)(AUV.travelTime/dt);
 RowVector2d  currentPosition(1,2);
 currentPosition= AUV.initialPosition;
 RowVector2d currentFlow;
 double  auxTime;
  int i=0;
while (i < steps ){
  //  auxTime=  AUV.cellTime.getTravelTime( -4,-4    );
    auxTime=  AUV.cellTime.getTravelTime( currentPosition(0,0),currentPosition(0,1)    );
 AUV.cellTime.setTraveltime(currentPosition(0,0),currentPosition(0,1) , auxTime +dt  );
currentFlow = FlowMap.getFlow(currentPosition(0,0),currentPosition(0,1) ) ;
//cout << "auxTime"<< auxTime<< "currentPosition(0,0)"<<currentPosition(0,0)<<"currentPosition(0,1)"  <<currentPosition(0,1)<< endl;

currentPosition=currentPosition+dt*(currentFlow+AUV.controlSpeed);
// cout <<"FlowMap.getFlow " <<FlowMap.getFlow(currentPosition(0,0),currentPosition(0,1) ) <<endl;
  //cout<<"currentPosition"<<currentPosition<<endl;
i++;
}
AUV.MTerror=AUV.finalPosition-currentPosition;
//cout<<"currentFlow"<<currentFlow<<endl;

 //cout <<"FlowMap.getFlow " <<FlowMap.getFlow(-4.5 ,  -5 ) <<endl;
return currentPosition;
}

MatrixXd simulatorTrajectory(  double &dt, vehicle  &AUV , CellStorage  &FlowMap ){
    int steps= (int)(AUV.travelTime/dt);
    
 MatrixXd trajectory(steps,2) ;
 RowVector2d  currentPosition(1,2);
 currentPosition= AUV.initialPosition;
 
 RowVector2d currentFlow;
 double  auxTime;
 double time=0;

  int i=0;
while (i < steps ){
   
    auxTime=  AUV.cellTime.getTravelTime( currentPosition(0,0),currentPosition(0,1)    );
  AUV.cellTime.setTraveltime(currentPosition(0,0),currentPosition(0,1) , auxTime  +dt  );
currentFlow = FlowMap.getFlow(currentPosition(0,0),currentPosition(0,1) ) ;
 

currentPosition=currentPosition+dt*(currentFlow+AUV.controlSpeed);
 
     
 trajectory(i,0)=currentPosition(0,0);
  trajectory(i,1)=currentPosition(0,1);
    
i++;
}

return trajectory;
}



int main(int, char**) {
int numberIteration=20;
double omega=0.5;
int numberAUVs=18;
int xDim=10;
int xMin=-5;
int xCellheight=1;
int yCellheight=1;
int yDim=10;
int yMin=-5;
int mapSize=xDim*yDim;
MatrixXd FlowVect (mapSize ,2);
MatrixXd TrueFlowVect (mapSize ,2);
MatrixXd FlowEstimationError (mapSize ,2);
MatrixXd travelTimeAUV (mapSize ,1);

MatrixXd travelTimeTotalAUV (mapSize,numberAUVs  );

CellStorage  FlowMap( xDim,  yDim,    xMin,   yMin,    xCellheight,   yCellheight) ;
CellStorage TrueFlowMap( xDim,  yDim,    xMin,   yMin,    xCellheight,   yCellheight) ;
RowVector2d initalFlow ;
RowVector2d controlSpeed;
initalFlow<< 0, 0; 
FlowMap.initializeFlow(initalFlow);
initalFlow<< 0.5, 0; 
TrueFlowMap.initializeFlow(initalFlow);
convertFlowMapToVector( TrueFlowMap,  TrueFlowVect ); 

double dt=0.1; 
MatrixXd MTerror (numberAUVs,2);
 
 
// Simulation Setup 
 
double dx, dy,posx, posy;
vector<vehicle*>  vehicleVec ;
 //numberAUVs
for (int i=0;i< numberAUVs;i++){
   
    
 vehicle *AUV= new vehicle() ;  
  cout<< "here"<<i <<endl;
    if (i <9){
        dx=1;
        dy=0;
        controlSpeed<< 0,0.5;
        posx= -5.5+(i+1)*dx;
        posy= -5;

    }else {
        dx=0;
        dy=1;
        controlSpeed<<  0.5,0;
        posx= -5;
        posy= -5.5+(i-7)*dy;

    }
    
 
 AUV->initialPosition   <<posx ,posy;
 AUV->controlSpeed << controlSpeed;
AUV->travelTime =2;
AUV->MTerror<<0,0;
AUV->finalPosition<<0,0;
cout<< "now"<<AUV->initialPosition  <<endl;
 AUV->finalPosition=simulator(   dt,  *AUV , TrueFlowMap );
 cout<< "pronS"<<i <<endl;
  AUV->cellTime.initializeTraveltime(0);

vehicleVec.push_back(AUV )   ;
 
 
}
 
 
 convertFlowMapToVector( FlowMap,  FlowVect ); 
 RowVector2d predictedFinalPosition;
int n=0;
//numberIteration
while ( n<numberIteration) { 
    n++;
    // Forward Step

for(int i=0;i<numberAUVs;i++){
 // vehicle AUV = *( (vehicleVec[i]))
    
      predictedFinalPosition =simulator(   dt,  *( (vehicleVec[i])) , FlowMap );
      
   // (vehicleVec[i])->MTerror=(vehicleVec[i])->finalPosition-predictedFinalPosition;
 
    MTerror.row(i)=(vehicleVec[i])->MTerror;
           
    convertTimeMap( (vehicleVec[i])->cellTime ,   travelTimeAUV );
   
travelTimeTotalAUV.col(i)=travelTimeAUV ;

}
   
 // Inverse Step
 double  inversNorm=1/ travelTimeTotalAUV.norm();
 FlowVect=FlowVect+0.5* (inversNorm*inversNorm ) *travelTimeTotalAUV* MTerror;
//cout<<travelTimeTotalAUV <<endl;
 
cout<<"MT error" <<MTerror.norm()<<endl;

 
 

//
convertVectorFlowToMap( FlowMap,  FlowVect ); 

 
FlowEstimationError=TrueFlowVect-FlowVect;
//cout<<travelTimeTotalAUV<<endl;
 cout << "Erstimation Error"<<FlowEstimationError.norm() <<endl;
}

cout << " predictedFinalPosition" <<predictedFinalPosition <<endl;
 
  cout << " vehicleVec[18]->initialPosition" <<vehicleVec[17]->initialPosition<<endl;
cout <<travelTimeAUV <<endl;
cout<<"MT error" <<MTerror <<endl;
   
   cout << "FlowVect"<<FlowVect <<endl;



Gnuplot gp;
	// Create a script which can be manually fed into gnuplot later:
	//    Gnuplot gp(">script.gp");
	// Create script and also feed to gnuplot:
	//    Gnuplot gp("tee plot.gp | gnuplot -persist");
	// Or choose any of those options at runtime by setting the GNUPLOT_IOSTREAM_CMD
	// environment variable.

	// Gnuplot vectors (i.e. arrows) require four columns: (x,y,dx,dy)
	std::vector<boost::tuple<double, double, double, double> > pts_A;

	// You can also use a separate container for each column, like so:
	std::vector<double> pts_B_x;
	std::vector<double> pts_B_y;
	std::vector<double> pts_B_dx;
	std::vector<double> pts_B_dy;

	// You could also use:
	//   std::vector<std::vector<double> >
	//   boost::tuple of four std::vector's
	//   std::vector of std::tuple (if you have C++11)
	//   arma::mat (with the Armadillo library)
	//   blitz::Array<blitz::TinyVector<double, 4>, 1> (with the Blitz++ library)
	// ... or anything of that sort

	for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
		double theta = alpha*2.0*3.14159;
		pts_A.push_back(boost::make_tuple(
			 cos(theta),
			 sin(theta),
			-cos(theta)*0.1,
			-sin(theta)*0.1
		));

		pts_B_x .push_back( cos(theta)*0.8);
		pts_B_y .push_back( sin(theta)*0.8);
		pts_B_dx.push_back( sin(theta)*0.1);
		pts_B_dy.push_back(-cos(theta)*0.1);
	}

	// Don't forget to put "\n" at the end of each line!
	gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "plot '-' with vectors title 'pts_A', '-' with vectors title 'pts_B'\n";
	gp.send1d(pts_A);
	gp.send1d(boost::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));

#ifdef _WIN32
	// For Windows, prompt for a keystroke before the Gnuplot object goes out of scope so that
	// the gnuplot window doesn't get closed.
	std::cout << "Press enter to exit." << std::endl;
	std::cin.get();
#endif


////Plot 

/*

int num = 4;                                        // number of data items
   int x[] = { 10, 50, 100, 200 };
   int y[] = {  3,  7,  19,  25 };

   // Write to file
   ofstream fileOut( "file.txt" );                     // open file
   
   fileOut << "#arrSize Values" << endl;               // write a header (not strictly necessary for gnuplot)
   
   for ( int i = 0; i < num; i++ )                     // loop round, dealing with each line of data
   {
      fileOut << x[i] << "  " << y[i] << endl;         // gnuplot happy with space between items; remember newline
   }
   
   fileOut.close(); 

  Gnuplot gp;
  gp << "plot \"file.txt\" 1:2" ;
 



std::fstream myfile;

myfile.open("test_gnuplot_data.dat");

myfile << "test_gnuplot_data.dat" << std::endl;
for (double x = 0; x<10; x+=0.1)
{
    myfile  << x << ", " << sin(x) << ", "  << cos(x) << "\n";
}

myfile.close();

 Gnuplot gp;
  gp << "plot \"test_gnuplot_data.dat\" 1:2" ;
 plot "test_gnuplot_data.dat";
   
 


chronr::ChStreamOutAsciiFile mdatafile("test_gnuplot_data.dat");
for (double x = 0; x<10; x+=0.1)
   mdatafile << x << ", " << sin(x) << ", "  << cos(x) << "\n";
   */
     
}
 