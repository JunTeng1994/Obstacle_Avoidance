#include "segment.h"

Segment::Segment():seedpoint(0, 0, 0), filterparam(0.01), obsthred(20), robtolerance(0.05)
{
	obsnum = 0;
	flag = false;
	cloud = PointCloudT::Ptr(new PointCloudT);
}

Segment::Segment(PointT point, float tolerance, float param, int thred)
{
	seedpoint.x = point.x;
	seedpoint.y = point.y;
	seedpoint.z = point.z;

	robtolerance = tolerance;
	filterparam = param;
	obsthred = thred;
	obsnum = 0;

	flag = false;

	cloud = PointCloudT::Ptr(new PointCloudT);
}

Segment::~Segment()
{

}

void Segment::VoxelGridFilter(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout)
{

	//std::cout << "PointCloud before filtering has: " << cloudin->points.size() << " data points." << std::endl;
	pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(filterparam, filterparam, filterparam);
	voxel.setInputCloud(cloudin);
	voxel.filter(*cloudout);
	//std::cout << "PointCloud after filtering has: " << cloudout->points.size() << " data points." << std::endl;
}

void Segment::OutlierRemovalFilter(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloudin);
	sor.setMeanK(15);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloudout);
}

void Segment::DeleteRobot(PointCloudT::Ptr cloudin, PointCloudT::Ptr& cloudout)
{
	// Creating the KdTree object for the search method of the extraction
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	cloudin->push_back(seedpoint);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloudin);

	// Check if the tree is sorted -- if it is we don't need to check the first element
	int nn_start_idx = tree->getSortedResults() ? 1 : 0;

	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed(cloudin->points.size(), false);

	std::vector<int> seed_queue;
	seed_queue.push_back(static_cast<int>(cloudin->points.size()) - 1);
	int sq_idx = 0;

	while (sq_idx < static_cast<int> (seed_queue.size()))
	{
		// Search for sq_idx
		if (!tree->radiusSearch(cloudin->points[seed_queue[sq_idx]], robtolerance, nn_indices, nn_distances))
		{
			sq_idx++;
			continue;
		}
		for (size_t j = nn_start_idx; j < nn_indices.size();++j)
		{
			if (nn_indices[j] == -1 || processed[nn_indices[j]] || nn_indices[j] == static_cast<int>(cloudin->points.size()) - 1)
				continue;

			// Perform a simple Euclidean clustering
			seed_queue.push_back(nn_indices[j]);
			processed[nn_indices[j]] = true;
		}
		sq_idx++;
	}

	cloudout->width = cloudin->size() - seed_queue.size();
	cloudout->height = 1;
	cloudout->is_dense = true;

	sort(seed_queue.begin(), seed_queue.end());
	std::vector<int>::iterator it = seed_queue.begin();
	int index_in = 0;
	for (;index_in < cloudin->size();index_in++)
	{
		if (index_in == *it)
		{
			if (it != seed_queue.end() - 1)
				it++;
			continue;
		}
		cloudout->push_back(cloudin->points[index_in]);
	}
}

void Segment::SegmentObstacle(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
}

void Segment::ComputeCube(PointCloudT::Ptr cloudin, CubePostion& cube)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*cloudin, *cloud);

	pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> nor;

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	nor.setSearchMethod(tree);
	nor.setInputCloud(cloud);
	nor.setNumberOfThreads(10);
	nor.setRadiusSearch(0.03);
	nor.compute(*cloud);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	transform.block<3, 1>(0, 3) = -1.f * (transform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloudWithNormals(*cloud, *transformedCloud, transform);

	pcl::PointXYZRGBNormal minPoint, maxPoint;
	pcl::getMinMax3D(*transformedCloud, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	cube.rotation = eigenVectorsPCA;
	cube.translation = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
	cube.width = maxPoint.x - minPoint.x;
	cube.height = maxPoint.y - minPoint.y;
	cube.depth = maxPoint.z - minPoint.z;
}

void Segment::ObstacleRecognize()
{
	obsnum = 0;
	flag = false;

	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	VoxelGridFilter(cloud, cloud_filtered);
	//OutlierRemovalFilter(cloud_filtered, cloud_filtered);

	PointCloudT::Ptr cloud_delete(new PointCloudT);
	DeleteRobot(cloud_filtered, cloud_delete);

	//OutlierRemovalFilter(cloud_delete, cloud_delete);

	//pcl::PCDWriter writer;
	//writer.write<PointT>("cloud_delete.pcd", *cloud_delete, false);

	if (cloud_delete->size() > obsthred)
	{
		std::vector<pcl::PointIndices> cluster_indices;
		SegmentObstacle(cloud_delete, cluster_indices);

		if (cluster_indices.size() != 0)
		{
			flag = true;
			obsnum = cluster_indices.size();
			//std::cout << "The number of obstacles is " << obsnum << std::endl;
			cubes.resize(obsnum);

			int j = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
					cloud_cluster->points.push_back(cloud_delete->points[*pit]);
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				ComputeCube(cloud_cluster, cubes[j]);

				//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
				//std::stringstream ss;
				//ss << "cloud_cluster_" << j << ".pcd";
				//writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); 
				j++;
			}
		}
	}
}

void Segment::setCloud(PointCloudT::Ptr cloudin)
{
	pcl::copyPointCloud(*cloudin, *cloud);
}

std::vector<CubePostion> Segment::getCubes()
{
	return cubes;
}

bool Segment::getFlag()
{
	return flag;
}

int Segment::getObsNum()
{
	return obsnum;
}
