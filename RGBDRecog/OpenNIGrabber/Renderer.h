
#include "stdafx.h"

class Renderer
{
public:
	Renderer() : 
		viewer(NULL),
		renderDepth(false),
		renderRGB(true),
		renderXYZ(false),
		renderXYZRGB(false)
	{}

		// 
	bool renderDepth;
	bool renderRGB;
	bool renderXYZ;
	bool renderXYZRGB;
protected:
	pcl::visualization::CloudViewer* viewer;

private:
};