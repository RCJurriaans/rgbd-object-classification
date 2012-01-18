#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <ctime>

int
 main (int argc, char** argv)
{
	double x = 2.3453453;
	cout << x;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::io::loadPCDFile<pcl::PointXYZ> (".\\depth_1.pcd", *cloud);

  cout << "loaded" << endl;
  //pcl::visualization::CloudViewer* v = new pcl::visualization::CloudViewer("aaa");
  //v->showCloud(cloud);
 
  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

  std::cerr << "ModelCoefficients calculated\n ";

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  std::cerr << "Segmentation object made\n ";
  // Optional
  seg.setOptimizeCoefficients (false);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setMaxIterations(10);
  seg.setInputCloud (cloud->makeShared ());

  std::cerr << "Segmentation parameters set\n ";

  time_t start = time(NULL);

  seg.segment (*inliers, *coefficients);

  time_t end = time(NULL);
  double t = difftime(end,start);
  cout << "Time taken: " << t << "s" << endl;

//  std::cerr << "Segment made\n ";


  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	cin.get();
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //for (size_t i = 0; i < inliers->indices.size (); ++i)
  //  std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
  //                                             << cloud.points[inliers->indices[i]].y << " "
  //                                             << cloud.points[inliers->indices[i]].z << std::endl;

   pcl::ExtractIndices<pcl::PointXYZ> extract;
   //const boost::shared_ptr<
   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(&cloud);

   extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter (*outCloud);

	//delete v;
	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(outCloud);

	while (!viewer.wasStopped ())
	{
		Sleep(1000);
	}
  return (0);
}