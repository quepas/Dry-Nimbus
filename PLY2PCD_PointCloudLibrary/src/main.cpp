#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr input_ply (new pcl::PointCloud<pcl::PointNormal>);
  pcl::io::loadPLYFile<pcl::PointNormal>("./data/in.ply", *input_ply);
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinaryCompressed("./data/out.pcd", *input_ply);
}
