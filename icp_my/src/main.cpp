#include "scanContainer.h"
#include "icp.h"
#include "laserSimulation.h"

void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

int main()
{
	std::cout<<"--------------- Program Begins -------------------"<<std::endl;
	
	// 1. laser simulation instance
	slam::simulation::Simulation simulation;
	
	// 2. open the simulation file
	simulation.openSimulationFile( "../data/laser2.txt" );
	
	// 3. read the data
	slam::sensor::LaserScan scan1;
	slam::sensor::LaserScan scan2;

	slam::ScanContainer points1;
	slam::ScanContainer points2;

	simulation.readAFrameData( scan1 );
	simulation.readAFrameData( scan2 );

	simulation.closeSimulationFile();

	// 4. convert the scan to scanContainer
	laserData2Container( scan1, points1 );
	laserData2Container( scan2, points2 );
	
	// 5. display the scan data
	points1.displayAFrameScan( 1, 20 );
	points2.displayAFrameScan( 2, 20 );

	cv::waitKey(0);
	
	// 6. icp instance
	slam::ICP icp;
	
	// 7. solve the icp 
	icp.solveICP( points1, points2 );

	// 8. get the results
	Eigen::Matrix<float, 2, 2> rot = icp.getRotateMatrix();

	std::cout<<"R = "<<std::endl<<rot<<std::endl;
	
	std::cout<<"---------------------------------------"<<std::endl;

	Eigen::Vector2f t = icp.getTransform();
	
	std::cout<<"T = "<<std::endl<<t<<std::endl;

	std::cout<<"------------------ END ----------------"<<std::endl;

	

	return 0;
}
