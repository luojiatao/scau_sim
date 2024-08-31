/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "lidar_cluster.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

namespace ns_lidar_cluster {
// Constructor
LidarCluster::LidarCluster(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

// load Param
void LidarCluster::loadParameters() {
  getRawLidar_ = false;
  is_ok_flag_ = false;
}

// Getters
sensor_msgs::PointCloud LidarCluster::getLidarCluster() { return cluster_; }
// 发布拟合地面后分离出来的地面和非地面点云以在rviz中进行可视化调试
sensor_msgs::PointCloud2 LidarCluster::getfilter_ground_() { return filter_ground_; }
sensor_msgs::PointCloud2 LidarCluster::getfilter_cones_() { return filter_cones_; }

bool LidarCluster::is_ok() const { return is_ok_flag_; }

// Setters
void LidarCluster::setRawLidar(const sensor_msgs::PointCloud2 &msg) {
  raw_pc2_ = msg;
  getRawLidar_ = true;
}

void LidarCluster::runAlgorithm() {
  if (raw_pc2_.fields.empty() || !getRawLidar_) {
    return;
  }
  getRawLidar_ = false;

  pcl::fromROSMsg(raw_pc2_, raw_pc_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(
      new pcl::PointCloud<pcl::PointXYZI>);

  // 对点云进行预处理，将点云分割成地面点云和路锥点云
  preprocessing(raw_pc_, cloud_ground, cloud_cones);

  // 对路锥点云进行聚类处理
  ClusterProcessing(cloud_cones, 0.5);

  filter_ground_.header.frame_id = "/rslidar";
  filter_ground_.header.stamp = raw_pc2_.header.stamp;
  filter_cones_.header.frame_id = "/rslidar";
  filter_cones_.header.stamp = raw_pc2_.header.stamp;
  cluster_.header.frame_id = "/rslidar";
  cluster_.header.stamp = raw_pc2_.header.stamp;
  is_ok_flag_ = true;
}

void LidarCluster::preprocessing(
    pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones) {

  pcl::PointCloud<pcl::PointXYZI> filtered;

  std::cout << "过滤前的原始点云数量为: " << raw.points.size() << std::endl;
/*  预处理过滤点云：
首先，对原始点云中的每个点进行遍历，通过一些条件来过滤掉不感兴趣的点：
距离过滤：如果点在 x-y 平面上的距离小于 sqrt(2)（约 1.414 米），认为其过于接近车辆，被过滤掉。
高度过滤：如果点的 z 值大于 0.7 米，认为其过高，被过滤掉。
前后过滤：x < 0 的点被过滤掉，这意味着车辆后方的点不被处理。
远距离地面过滤：如果点在远距离（x-y 平面上的距离大于 7 米）且 z 值小于 0.03 米，被认为是远处的地面点，过滤掉。
通过上述条件筛选后的点被存入新的点云 filtered 中。*/

  for (auto &iter : raw.points) {
    if (std::hypot(iter.x, iter.y) < sqrt(2) || iter.z > 0.7 ||
        (std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03) || std::hypot(iter.x, iter.y) > 8)
    // if (std::hypot(iter.x, iter.y) < sqrt(2) || iter.z > 0.7 ||
    //     iter.x < 0 || (std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03))
      continue;
    filtered.points.push_back(iter);
  }
  std::cout << "预处理过滤后的点云数量为: " << filtered.points.size() << std::endl;

/*平面分割：
使用 PCL 的 SACSegmentation 对象进行平面分割。
seg.setModelType(pcl::SACMODEL_PLANE) 设置分割模型为平面模型。
seg.setMethodType(pcl::SAC_RANSAC) 使用随机采样一致性（RANSAC）算法来拟合平面。
seg.setDistanceThreshold(0.07) 设置距离阈值为 0.07 米，即点与拟合平面的距离小于 0.07 米时，被认为是属于平面的点。
seg.segment(*inliers, *coefficients) 执行分割操作，输出平面的内点（inliers）和平面系数（coefficients）。*/
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);     //coefficients 用于存储地面模型的参数（例如平面的法向量和截距）。
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);    //inliers 用于存储属于地面的点的索引。
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional（允许优化模型系数）
  seg.setOptimizeCoefficients(true);
  // Mandatory（必选的）
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // 距离阈值
  seg.setDistanceThreshold(0.07);
  seg.setInputCloud(filtered.makeShared());
  seg.segment(*inliers, *coefficients);

  /* Debug
    for (auto iter : coefficients->values) {
      std::cout << iter << " ";
    }
    std::cout << "\n-------------\n";
   */

  // extract ground
  pcl::ExtractIndices<pcl::PointXYZI> extract;    //pcl::ExtractIndices 是一个提取对象，用于根据索引提取点云中的特定点。
  extract.setInputCloud(filtered.makeShared());    // 设置输入点云为原始点云 raw。
  extract.setIndices(inliers);                                      //设置要提取的点的索引为地面点 inliers。
  extract.filter(*cloud_ground);                              //将提取的地面点存储到 cloud_ground 中。

  // extract cone
  extract.setNegative(true);                                      //反转提取逻辑，这样它会提取非地面点（即不属于 inliers 的点）。
  extract.filter(*cloud_cones);

  std::cout << "过滤掉地面后的锥桶点云数量为: " << cloud_cones->size() << std::endl;


  pcl::toROSMsg(*cloud_ground, filter_ground_);
  pcl::toROSMsg(*cloud_cones, filter_cones_);
}

void LidarCluster::ClusterProcessing( const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold) 
{

  cluster_.points.clear();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  std::cout << "输入聚类方法的点云数量: " << cloud->size() << std::endl;
  // 过滤无效点
  for (const auto& point : *cloud) {
      if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) &&
          !std::isinf(point.x) && !std::isinf(point.y) && !std::isinf(point.z)) {
          cloud_filtered->points.push_back(point);
      }
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);

  std::cout << "实际输入KD树的点云数量为: " << cloud_filtered->size() << std::endl;

// 欧几里得聚类：
    // 使用 PCL 的 EuclideanClusterExtraction 对象 ec 进行聚类提取。
    // 设置聚类的参数：
    // ec.setClusterTolerance(threshold)：设置聚类的容忍距离，即点与点之间的最大距离，如果小于这个距离，它们会被归为同一个簇。
    // ec.setMinClusterSize(2)：设置最小的聚类大小，防止误检测（通常是噪声点）被认为是有效的聚类。
    // ec.setMaxClusterSize(200)：设置最大的聚类大小。
    // ec.extract(cluster_indices) 执行聚类操作，cluster_indices 存储了每个聚类的点索引。

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(threshold);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  std::cout << "聚类后的结果为: " << cluster_indices.size() << std::endl;

// 处理每个聚类：遍历所有的聚类（即 cluster_indices），并为每个聚类创建一个新的点云对象 cone。
// 将聚类中的点（根据索引 iter.indices）添加到 cone 点云中，并设置点云的相关属性（宽度、高度和密度）。
  // int cluster_id = 0;    
  for (const auto &iter : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it : iter.indices) {
      cone->points.push_back(cloud_filtered->points[it]);
    }
    cone->width = cone->points.size();
    cone->height = 1;
    cone->is_dense = true;

  // 计算聚类特征：
    // 使用 pcl::compute3DCentroid 计算聚类的 3D 质心，存储在 centroid 中。
    // 使用 pcl::getMinMax3D 获取聚类的最小和最大 3D 边界值，分别存储在 min 和 max 中。
    // 计算聚类的边界大小 bound_x, bound_y, bound_z，分别表示聚类在 x、y 和 z 方向上的范围。
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::compute3DCentroid(*cone, centroid);
    pcl::getMinMax3D(*cone, min, max);

    float bound_x = std::fabs(max[0] - min[0]);
    float bound_y = std::fabs(max[1] - min[1]);
    float bound_z = std::fabs(max[2] - min[2]);

//基于形状的过滤：
  /* 通过判断聚类的边界大小是否符合路锥的特征来过滤聚类：
   bound_x < 0.5，bound_y < 0.5，bound_z < 0.4：限制聚类的边界范围，使其符合路锥的尺寸。
   centroid[2] < 0.4：限制聚类质心的 z 轴位置，保证其处于一定的高度以下。(这个值需要根据雷达安装位置来调，可以是负值)
   对于满足上述条件的聚类，提取其质心的坐标，存储为 ROS 点消息 geometry_msgs::Point32，
   并添加到 cluster_ 点云中，作为最终的输出。*/
    if (bound_x < 0.3 && bound_y < 0.3 && bound_z < 0.4 && centroid[2] < 0) {
      geometry_msgs::Point32 tmp;
      tmp.x = centroid[0];
      tmp.y = centroid[1];
      tmp.z = centroid[2];
      cluster_.points.push_back(tmp);

      // // 创建可视化窗口
      // pcl::visualization::PCLVisualizer viewer ("Cluster " + std::to_string(cluster_id));

      // // 添加点云
      // viewer.addPointCloud<pcl::PointXYZI> (cone, "cone_" + std::to_string(cluster_id));

      // // 设置颜色
      // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
      //                                           rand() % 255 / 255.0, 
      //                                           rand() % 255 / 255.0, 
      //                                           rand() % 255 / 255.0, 
      //                                           "cone_" + std::to_string(cluster_id));

      // // 开始可视化循环
      // while (!viewer.wasStopped ()) {
      //     viewer.spinOnce (100);
      //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      // }
    }
  }
}

} // namespace ns_lidar_cluster
