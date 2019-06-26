#include "costmap.hpp"
#include <iostream>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace octomap;

costmap::costmap(double resolution)
{
	octree = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(resolution));
	cout << "OcTree Created" <<endl;
}

int costmap::add_cloud(std::string path, double x, double y, double z, double yaw, double pitch, double roll)
{
	pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud);
	cloud_origin = octomap::pose6d(x,y,z,roll,pitch,yaw);

	vector<int> indexes;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	pcl::removeNaNFromPointCloud(cloud, filtered_cloud, indexes);

	cout << "filtered size: " << filtered_cloud.size() << endl;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	pcl::getTransformation(x,y,z,roll,pitch,yaw, transform);
	transform = transform.inverse();

	//pcl::transformPointCloud(filtered_cloud, filtered_cloud, transform);


	octomap::KeySet free_cells, occupied_cells;
	octomap::KeyRay key_ray;
	point3d origin(x,y,z);

	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = filtered_cloud.begin(); it != filtered_cloud.end(); ++it){
		point3d point(it->x, it->y, it->z);

		if (octree->computeRayKeys(origin, point, key_ray)){
			free_cells.insert(key_ray.begin(), key_ray.end());
		}

		OcTreeKey key;
    	if (octree->coordToKeyChecked(point, key)){
      		occupied_cells.insert(key);
    	}
	}

	// mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    octree->updateNode(*it, true);
  }



}





int costmap::read_cloud(std::string path, double x, double y, double z, double yaw, double pitch, double roll)
{
	pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud);
    cloud_origin = octomap::pose6d(x,y,z,roll,pitch,yaw);

}

int costmap::write_octomap(std::string path)
{
  octree->writeBinary(path);
}

int costmap::insert_cloud(pcl::PointCloud<pcl::PointXYZ> cloud,  double x, double y, double z, double yaw, double pitch, double roll)
{
	vector<int> indexes;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	pcl::removeNaNFromPointCloud(cloud, filtered_cloud, indexes);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	pcl::getTransformation(x,y,z,roll,pitch,yaw, transform);
	//transform = transform.inverse();

	pcl::transformPointCloud(filtered_cloud, filtered_cloud, transform);


	octomap::Pointcloud octocloud;

	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = filtered_cloud.begin(); it != filtered_cloud.end(); ++it) {
		point3d point(it->x, it->y, it->z);

		octocloud.push_back(point);
	}
	point3d sensor_origin(x,y,z);

	octree->insertPointCloud(octocloud, sensor_origin);

}

int costmap::test(std::string cloud_path, double x, double y, double z, double yaw, double pitch, double roll)
{
	read_cloud(cloud_path, x,y,z,yaw,pitch,roll);

	insert_cloud(cloud, x,y,z,roll,pitch,yaw);

}


int costmap::update()
{
	octomap::KeySet free_cells, occupied_cells;
	octomap::KeyRay key_ray;
	point3d origin(0,0,0);

	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
		point3d point(it->x, it->y, it->z);

		if (octree->computeRayKeys(origin, point, key_ray)){
			free_cells.insert(key_ray.begin(), key_ray.end());
		}

		OcTreeKey key;
    if (octree->coordToKeyChecked(point, key)){
      occupied_cells.insert(key);
    }
	}

	// mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    octree->updateNode(*it, true);
  }

}