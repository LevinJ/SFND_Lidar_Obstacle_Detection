/*
 * ProcessPointCloudsCourse.h
 *
 *  Created on: Jun 25, 2019
 *      Author: levin
 */

#ifndef SRC_PROCESSPOINTCLOUDSCOURSE_H_
#define SRC_PROCESSPOINTCLOUDSCOURSE_H_


#include "processPointClouds.h"

template<typename PointT>
class ProcessPointCloudsCourse: public ProcessPointClouds<PointT> {
public:
	ProcessPointCloudsCourse();
	virtual std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
	virtual ~ProcessPointCloudsCourse();
};

#endif /* SRC_PROCESSPOINTCLOUDSCOURSE_H_ */
