/*
 * ProcessPointCloudsCourse.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: levin
 */

#include "ProcessPointCloudsCourse.h"
#include <unordered_set>

template<typename PointT>
ProcessPointCloudsCourse<PointT>::ProcessPointCloudsCourse() {
	// TODO Auto-generated constructor stub

}

template<typename PointT>
ProcessPointCloudsCourse<PointT>::~ProcessPointCloudsCourse() {
	// TODO Auto-generated destructor stub
}

template<typename PointT>
void ProcessPointCloudsCourse<PointT>::RansacPlane(typename  pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol,
		pcl::PointIndices &inliers, pcl::ModelCoefficients &model_coefficients)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

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
	return;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>  ProcessPointCloudsCourse<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
	// Create the segmentation object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::SACSegmentation<PointT> seg;


	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = this->SeparateClouds(inliers,cloud);
    return segResult;
}


