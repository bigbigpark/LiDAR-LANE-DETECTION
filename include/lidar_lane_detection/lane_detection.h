/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-14 20:17
 */
#pragma once

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>

// PCL for transformation
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <ctime>
#include <time.h>
#include <chrono>

#include <lidar_msgs/Lane.h>

using namespace std;

#define OS1_HEIGHT 1.9

class LaneDetector
{
public:
  LaneDetector();
  ~LaneDetector();
public:
  bool init();

public:
  void OS1PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs);
  pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr& inliers, const  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceThreshold);

private:
  ros::Publisher origin_pub_;
  ros::Publisher lane_pub_;
  ros::Subscriber os1_sub_;
  ros::NodeHandle nh_;

  sensor_msgs::PointCloud2 ros_input_;
  sensor_msgs::PointCloud2 ros_output_;

  ros::Publisher lane_udp_pub_;

  float theta_r;
};