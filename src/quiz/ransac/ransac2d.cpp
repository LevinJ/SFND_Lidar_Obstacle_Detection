/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--){
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()<2){
			inliers.insert(rand()%cloud->points.size());
		}
		// Measure distance between every point and fitted line
		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		it++;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = x1 * y2 - x2 * y1;
		// If distance is smaller than threshold count it as inlier
		for(int i=0; i < cloud->points.size(); i++){
			if(inliers.count(i)> 0){
				continue;
			}
			float x = cloud->points[i].x;
			float y = cloud->points[i].y;
			float d = fabs(a * x + b * y + c)/sqrt(a * a + b*b);
			if (d < distanceTol){
				inliers.insert(i);
			}
		}


		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}

	
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol,
		pcl::PointIndices &inliers, pcl::ModelCoefficients &model_coefficients)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while(maxIterations--){
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()<3){
			inliers.insert(rand()%cloud->points.size());
		}
		// Measure distance between every point and fitted line
		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		float z1 = cloud->points[*it].z;
		it++;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;
		float z2 = cloud->points[*it].z;
		it++;
		float x3 = cloud->points[*it].x;
		float y3 = cloud->points[*it].y;
		float z3 = cloud->points[*it].z;

		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);


		float a = i;
		float b = j;
		float c = k;
		float d = -(i*x1+j*y1+k*z1);
		// If distance is smaller than threshold count it as inlier
		for(int i=0; i < cloud->points.size(); i++){
			if(inliers.count(i)> 0){
				continue;
			}
			float x = cloud->points[i].x;
			float y = cloud->points[i].y;
			float z = cloud->points[i].z;

			float dist = fabs(a * x + b * y + c*z + d)/sqrt(a * a + b*b + c*c);
			if (dist < distanceTol){
				inliers.insert(i);
			}
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
			model_coefficients.values = {a,b,c,d};
		}

	}

	//fill in output variable
	for(int point : inliersResult){
		inliers.indices.push_back(point);
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
//	std::unordered_set<int> inliers = Ransac(cloud, 10, 1);
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2, *inliers_1, *coefficients);
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(1,0,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,1,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	std::cerr << "PointCloud representing the planar component: " << inliers.size() << " data points." << std::endl;
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
