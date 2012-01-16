#include <stdio.h>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <math.h>

#include <pcl/range_image/range_image.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;

  std::string path;
  /*
	std::cout << "Enter pcd file path" << std::endl;
	std::cin >> path;*/
  path = "Saves/depth_2.pcd";
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud);
 
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

  std::cout << rangeImage << "\n";

}