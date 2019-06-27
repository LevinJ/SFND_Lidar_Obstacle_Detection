/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <chrono>
#include <string>
#include "kdtree.h"
#include <iostream>



void clusterHelper(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int i, std::vector<int> &cluster, std::vector<bool> &processed){
	if(processed[i]) return;
	cluster.push_back(i);
	processed[i] = true;

	std::vector<int> nearby = tree->search(points[i], distanceTol);
	for(auto &j: nearby){
		clusterHelper(points, tree, distanceTol, j, cluster, processed);
	}
}


void euclideanClusterWrapper(const std::vector<std::vector<float>>& points,  float clusterTolerance, int minSize, int maxSize, std::vector<std::vector<int>> &clusters)
{
	if(points.size() < minSize){
		return;
	}
	KdTree tree;
	tree.dim = points[0].size();

	for (int i=0; i<points.size(); i++)
		tree.insert(points[i],i);

	// Fill out this function to return list of indices for each cluster
	std::vector<bool> processed(points.size(), false);
	for(int i=0; i< points.size(); i++){
		if(processed[i]) continue;
		std::vector<int> cluster;
		clusterHelper(points, &tree, clusterTolerance, i, cluster, processed);
		//fitler out cluster whose size is not within range
		uint cluster_size = cluster.size();
		if(cluster_size < minSize || cluster_size > maxSize){
//			std::cout<<"discarded cluster "<<cluster_size<<std::endl;
			continue;
		}
		clusters.push_back(cluster);
	}
	return;
}
