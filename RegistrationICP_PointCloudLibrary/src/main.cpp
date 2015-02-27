#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::IterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::io::loadPLYFile;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PCLVisualizer;

int main (int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr cloud_base(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_target(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_base_aligned(new PointCloud<PointXYZ>);

  // load point clouds/meshes
  loadPLYFile("../data/human_1.ply", *cloud_base);
  loadPLYFile("../data/human_2.ply", *cloud_target);

  // display point clouds before registration
  PointCloudColorHandlerCustom<PointXYZ> base_color(cloud_base, 0, 0, 255);
  PointCloudColorHandlerCustom<PointXYZ> target_color(cloud_target, 255, 0, 0);
  PCLVisualizer* viewer = new PCLVisualizer("Point clouds BEFORE registration");
  viewer->addPointCloud(cloud_base, base_color, "cloud_base");
  viewer->addPointCloud(cloud_target, target_color, "cloud_target");
  while (!viewer->wasStopped ()) { viewer->spinOnce (100); }
  viewer->close();
  delete viewer;

  // register point cloud using iterative closest point algorithm (ICP)
  std::cout << "ICP started..." << std::endl;
  IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setInputCloud(cloud_base);
  icp.setInputTarget(cloud_target);
  icp.align(*cloud_base_aligned);

  // display converged stats
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  // display point clouds after registration
  PointCloudColorHandlerCustom<PointXYZ> base_aligned_color (cloud_base_aligned, 0, 0, 255);
  viewer = new PCLVisualizer("Point clouds AFTER registration");
  viewer->addPointCloud(cloud_target, target_color, "cloud_target");
  viewer->addPointCloud(cloud_base_aligned, base_aligned_color, "cloud_base_aligned");
  while (!viewer->wasStopped ()) { viewer->spinOnce (100); }
}
