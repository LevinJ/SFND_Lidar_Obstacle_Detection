// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new  pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (filterRes, filterRes, filterRes);
	sor.filter (*cloud_filtered);

	//region of interest
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new  pcl::PointCloud<PointT>());
	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloud_filtered);
	region.filter(*cloud_region);


	//clear the roof
	std::vector<int> indices;
	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
	roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
	roof.setInputCloud(cloud_region);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

	for(int point : indices){
		inliers->indices.push_back(point);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_region);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "After filtering " << cloud_region->points.size () << std::endl;
    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	pcl::ExtractIndices<PointT> extract;

	typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>);
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_p);
	std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	typename pcl::PointCloud<PointT>::Ptr cloud_o(new pcl::PointCloud<PointT>);
	extract.setNegative (true);
	extract.filter (*cloud_o);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_o, cloud_p);
    return segResult;
}

template<typename PointT>
float ProcessPointClouds<PointT>::pnt2plane_distance(pcl::ModelCoefficients coef){
	float a = coef.values[0];
	float b = coef.values[1];
	float c = coef.values[2];
	float d = coef.values[3];

	float x=0,y=0,z=0;

	float distance = fabs(a * x + b * y + c*z + d)/sqrt(a * a + b*b + c*c);
	return distance;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	// TODO:: Fill in this function to find inliers for the cloud.
	// Create the segmentation object
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

	//pcl method
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

	std::cout << "plane coefficients\n" << *coefficients<< std::endl;
	std::cout << "plane distance " << pnt2plane_distance(*coefficients)<< std::endl;

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = this->SeparateClouds(inliers,cloud);
	return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterTolerance); // 2cm
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);


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


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cout << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
