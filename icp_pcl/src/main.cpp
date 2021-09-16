#include "laserSimulation.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>

#include <limits>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void laserData2PointCloud( const slam::sensor::LaserScan &scan, const PointCloud::Ptr &pointCloud )
{

	float angle = -3.14159f;
	
	for( int i = 0; i < 1440; i ++ ){
		float dist = scan.ranges[i];
		
		PointT point_tmp;
		if( dist >= 0.0099999998f && dist <= 15.0000000000f ){
			point_tmp.x = cos( angle ) * dist;
			point_tmp.y = sin( angle ) * dist;
			point_tmp.z = 0.0;
		
			pointCloud->points.push_back( point_tmp );
		}
		
		angle += 0.0043633231f;
	} 
	
	pointCloud->width = pointCloud->size();
	pointCloud->height = 1;
	pointCloud->is_dense = true;

	std::cout<<"point cloud size: "<<pointCloud->size()<<std::endl;
}

int main()
{
        std::cout<<"--------------- Program Begins -------------------"<<std::endl;

	PointCloud::Ptr pointCloud1( new PointCloud );
	PointCloud::Ptr pointCloud2( new PointCloud );

	// 1. laser simulation instance
        slam::simulation::Simulation simulation;

        // 2. open the simulation file
        simulation.openSimulationFile( "../data/laser2.txt" );

        // 3. read the data
        slam::sensor::LaserScan scan1;
        slam::sensor::LaserScan scan2;

        simulation.readAFrameData( scan1 );
        simulation.readAFrameData( scan2 );

        simulation.closeSimulationFile();

	// 4. convert the scan to point cloud
	laserData2PointCloud( scan1, pointCloud1 );
	laserData2PointCloud( scan2, pointCloud2 );
	
	// 5. save the pcd files
	pcl::io::savePCDFileBinary( "scan1.pcd", *pointCloud1 );
	pcl::io::savePCDFileBinary( "scan2.pcd", *pointCloud2 );

	// 6. icp by pcl library
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations (100);

	icp.setInputSource( pointCloud1 );
    	icp.setInputTarget( pointCloud2 );

	// 7. iterate
	pcl::PointCloud<pcl::PointXYZ> unused_result;
	icp.align( unused_result );

	if( icp.hasConverged() == false ){
		std::cout<<"Not Converged ..."<<std::endl;

	}
	else{
		Eigen::Affine3f transform;
		transform = icp.getFinalTransformation();
		
		//std::cout<<"transform: "<<std::endl<<transform<<std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		std::cout<<std::endl;	

		float x, y, z, roll, pitch, yaw;
		pcl::getTranslationAndEulerAngles( transform, x, y, z, roll, pitch, yaw );
		//std::cout<<"transform: ( "<<x<<", "<<y<<", "<<yaw * 180 / M_PI<<" )"<<std::endl;
	
		std::cout<<"transform: ( "<<x<<", "<<y<<", "<<yaw<<" )"<<std::endl;
	}

	return 0;
}
