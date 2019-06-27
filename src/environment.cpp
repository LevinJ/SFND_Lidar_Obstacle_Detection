/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "ProcessPointCloudsCourse.h"
#include "ProcessPointCloudsCourse.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
//    renderRays(viewer, lidar->position, inputCloud);
//    renderPointCloud(viewer, inputCloud, "pc1", Color(0,1,0));
    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(0,1,1));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
//          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          Box box = pointProcessor.BoundingBox(cluster);
          renderBox(viewer,box,clusterId);
          ++clusterId;
    }


  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

//  renderPointCloud(viewer,inputCloud,"inputCloud");
  Box box;
  box.x_min = -10;
  box.y_min = -6;
  box.z_min = -2;
  box.x_max = 30;
  box.y_max = 7;
  box.z_max = 3;
//  renderBox(viewer, box, 1, Color(0,1,0), 1);
  auto filterCloud = pointProcessor->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (box.x_min, box.y_min,  box.z_min, 1),
		  Eigen::Vector4f ( box.x_max, box.y_max , box.z_max, 1));
  renderPointCloud(viewer,filterCloud,"filterCloud");


  //the box for filtering the roof area
  box.x_min = -1.5;
  box.y_min = -1.7;
  box.z_min = -1;
  box.x_max = 2.6;
  box.y_max = 1.7;
  box.z_max = -0.4;
//  renderBox(viewer, box, 100, Color(0,1,0), 1);

  //segment the cloud
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 100, 0.2);
  renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


  //cluster the cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.5, 10, 1600);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
		renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
		Box box = pointProcessor->BoundingBox(cluster);
		renderBox(viewer,box,clusterId);
		++clusterId;
  }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);

//    cityBlock(viewer);
//    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    //ProcessPointCloudsCourse uses the RANSAC, KDTreea and Eucleadian clustering implemented by myself
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointCloudsCourse<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
    	// Clear viewer
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		// Load pcd and run obstacle detection process
		inputCloudI = pointProcessor->loadPcd((*streamIterator).string());
		cityBlock(viewer, pointProcessor, inputCloudI);

		streamIterator++;
		if(streamIterator == stream.end())
		{
			 std::cout << "END OF PCD STREAM" << std::endl;
			streamIterator = stream.begin();
		}

//		break;

        viewer->spinOnce ();
    } 

//    while (!viewer->wasStopped ()){
//    	viewer->spinOnce ();
//    }
}
