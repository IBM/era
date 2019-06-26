/*
 * Copyright 2018 IBM
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class costmap
{
public:

 	std::unique_ptr<octomap::OcTree> octree;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	octomap::pose6d cloud_origin;

	costmap(double resolution);
	int update();
	int read_cloud(std::string path, double x, double y, double z, double yaw, double pitch, double roll);
	int write_octomap(std::string path);
	int insert_cloud(pcl::PointCloud<pcl::PointXYZ> cloud,  double x, double y, double z, double yaw, double pitch, double roll);
	int test(std::string cloud_path, double x, double y, double z, double yaw, double pitch, double roll);
	int add_cloud(std::string path, double x, double y, double z, double yaw, double pitch, double roll);
};








