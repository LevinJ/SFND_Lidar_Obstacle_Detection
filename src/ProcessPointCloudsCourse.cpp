/*
 * ProcessPointCloudsCourse.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: levin
 */

#include "ProcessPointCloudsCourse.h"
#include <unordered_set>

void euclideanClusterWrapper(const std::vector<std::vector<float>>& points,  float clusterTolerance,
		int minSize, int maxSize, std::vector<std::vector<int>> &clusters);

template<typename PointT>
ProcessPointCloudsCourse<PointT>::ProcessPointCloudsCourse() {
	// TODO Auto-generated constructor stub

}

template<typename PointT>
ProcessPointCloudsCourse<PointT>::~ProcessPointCloudsCourse() {
	// TODO Auto-generated destructor stub
}

template<typename PointT>
void ProcessPointCloudsCourse<PointT>::RansacPlane(typename  pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceTol,
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

	//try fitting a more stable plane out of inliers
//	typename  pcl::PointCloud<PointT>::Ptr filter_cloud (new pcl::PointCloud<PointT>);
//	pcl::ExtractIndices<PointT> extract;
//	extract.setInputCloud (cloud);
//	pcl::PointIndices::Ptr pinliers (new pcl::PointIndices);
//	pinliers->indices = inliers.indices;
//	extract.setIndices (pinliers);
//	extract.setNegative (false);
//	extract.filter (*filter_cloud);
//
//	pcl::SACSegmentation<PointT> seg;
//	seg.setOptimizeCoefficients (true);
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_LMEDS);
//	seg.setMaxIterations (maxIterations);
//	seg.setDistanceThreshold (distanceTol);
//	// Segment the largest planar component from the remaining cloud
//	seg.setInputCloud (filter_cloud);
//	seg.segment (inliers, model_coefficients);

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

	RansacPlane(cloud, maxIterations, distanceThreshold, *inliers, *coefficients);

	//pcl method
//	pcl::SACSegmentation<PointT> seg;
//	seg.setOptimizeCoefficients (true);
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setMaxIterations (maxIterations);
//	seg.setDistanceThreshold (distanceThreshold);
//	// Segment the largest planar component from the remaining cloud
//	seg.setInputCloud (cloud);
//	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "plane coefficients\n" << *coefficients<< std::endl;
    std::cout << "plane distance " << this->pnt2plane_distance(*coefficients)<< std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = this->SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointCloudsCourse<PointT>::euclidian_clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance,
			int minSize, int maxSize, std::vector<pcl::PointIndices> &cluster_indices){
	std::vector<std::vector<float>> points;
	std::vector<std::vector<int>> clusters;
	//prepare the poitns in the form of std::vector<std::vector<float>>
	for(PointT &p : cloud->points){
		points.push_back({p.x, p.y, p.z});
	}
	euclideanClusterWrapper(points,  clusterTolerance, minSize, maxSize, clusters);

	for(auto cluster: clusters){
		pcl::PointIndices indices;
		indices.indices = std::move(cluster);
		cluster_indices.push_back(indices);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointCloudsCourse<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
		float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<pcl::PointIndices> cluster_indices;

    euclidian_clustering(cloud, clusterTolerance, minSize, maxSize, cluster_indices);

    //pcl method
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//	tree->setInputCloud (cloud);
//	pcl::EuclideanClusterExtraction<PointT> ec;
//	ec.setClusterTolerance (clusterTolerance); // 2cm
//	ec.setMinClusterSize (minSize);
//	ec.setMaxClusterSize (maxSize);
//	ec.setSearchMethod (tree);
//	ec.setInputCloud (cloud);
//	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		  cloud_cluster->points.push_back (cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		clusters.push_back(cloud_cluster);

	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

