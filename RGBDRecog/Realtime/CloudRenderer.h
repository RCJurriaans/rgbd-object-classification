
#include "stdafx.h"


class CloudRenderer
{
public:

	CloudRenderer() :
	  viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
	{
		viewer->setBackgroundColor (0, 0, 0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

};