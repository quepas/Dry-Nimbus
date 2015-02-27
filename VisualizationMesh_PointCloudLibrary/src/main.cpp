#include <iostream>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::PolygonMesh;
using pcl::io::loadPolygonFilePLY;
using pcl::visualization::PCLVisualizer;

int main()
{
  PolygonMesh human_1_mesh;
  loadPolygonFilePLY("../data/human_1.ply", human_1_mesh);
  PCLVisualizer* viewer = new PCLVisualizer("Mesh viewer");
  viewer->addPolygonMesh(human_1_mesh);
  while (!viewer->wasStopped()) { viewer->spinOnce(100); }
  viewer->close();
  delete viewer;
}
