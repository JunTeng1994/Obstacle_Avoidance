#pragma once

#define PCL_NO_PRECOMPILE  

#include "stdafx.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h> 
#include <boost/thread/thread.hpp> 
#include <Eigen/Core>
#include <pcl/console/time.h>


struct CubePostion
{
	Eigen::Vector3f translation = Eigen::Vector3f::Zero();
	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
	double width = 0;
	double height = 0;
	double depth = 0;
};

class Segment
{
public:

	Segment();
	Segment(PointT point, float tolerance, float param, int thred);
	~Segment();
	
	void ObstacleRecognize();

	void setCloud(PointCloudT::Ptr cloudin);

	std::vector<CubePostion> getCubes();

	bool getFlag();

	int getObsNum();
private:

	bool flag;
	std::vector<CubePostion> cubes;
	PointCloudT::Ptr cloud;
	PointT seedpoint;
	float robtolerance;
	float filterparam;
	int obsthred;
	int obsnum;

	void VoxelGridFilter(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout);

	void OutlierRemovalFilter(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout);

	void DeleteRobot(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout);

	void SegmentObstacle(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices);

	void ComputeCube(PointCloudT::Ptr cloudin, CubePostion& cube);

};