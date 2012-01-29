
#include "stdafx.h"
#include "RenderThread.h"

#include <algorithm>
using namespace std;

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	/*cout <<"keyboard cb: " << (int)e.getKeyCode() << endl;
	RenderThread* r = static_cast<RenderThread*>(cookie);
	
	switch( e.getKeyCode() ) {
	case 97:
		r->leftPressed = r->leftPressed == false;
		break;
	case 119:
		r->upPressed = r->upPressed == false;
		break;
	case 100:	
		r->rightPressed = e.keyDown();
		break;
	case 115:
		r->downPressed = r->downPressed == false;
		break;
	}
	r->shiftPressed = e.isShiftPressed();*/
	
	//boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	//if(e.keyDown()) {
	//	(*processInput)(e.getKeyCode());
	//}
}

float clamp(float val, float min, float max) {
	if( val < min ) return min;
	if( val > max ) return max;
	return val;
}


inline void addLine( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float x1, float y1, float z1, float x2, float y2, float z2, int r = 255, int g = 255, int b = 255)
{
	float dx = x2 - x1;
	float dy = y2 - y1;
	float dz = z2 - z1;
	int numpts = sqrt(dx*dx+dy*dy+dz*dz) * 100;
	for( int i = 0; i < numpts; i++ ) {

		pcl::PointXYZRGB pt;
		pt.x = x1 + dx * ((float)i / (numpts-1));
		pt.y = y1 + dy * ((float)i / (numpts-1));
		pt.z = z1 + dz * ((float)i / (numpts-1));

		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		pt.rgb = *reinterpret_cast<float*>(&rgb);

		cloud->push_back(pt);
	}
}

void addBox( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::ModelCoefficients& m, int r = 255, int g = 255, int b = 255)
{
	// Corners (center plus and minus width)
	float xp = m.values[0] + m.values[7] * 0.5f;
	float xm = m.values[0] - m.values[7] * 0.5f;
	float yp = m.values[1] + m.values[8] * 0.5f;
	float ym = m.values[1] - m.values[8] * 0.5f;
	float zp = m.values[2] + m.values[9] * 0.5f;
	float zm = m.values[2] - m.values[9] * 0.5f;

	// Lines between corners
	addLine( cloud, xp, yp, zp, xm, yp, zp, r, g, b );
	addLine( cloud, xp, yp, zp, xp, ym, zp, r, g, b );
	addLine( cloud, xp, yp, zp, xp, yp, zm, r, g, b );
	addLine( cloud, xm, yp, zm, xm, yp, zp, r, g, b );
	addLine( cloud, xm, yp, zm, xp, yp, zm, r, g, b );
	addLine( cloud, xm, yp, zm, xm, ym, zm, r, g, b );
	addLine( cloud, xm, ym, zm, xm, ym, zp, r, g, b );
	addLine( cloud, xm, ym, zm, xp, ym, zm, r, g, b );
	addLine( cloud, xp, ym, zm, xp, yp, zm, r, g, b );
	addLine( cloud, xm, ym, zp, xm, yp, zp, r, g, b );
	addLine( cloud, xp, ym, zm, xp, ym, zp, r, g, b );
	addLine( cloud, xm, ym, zp, xp, ym, zp, r, g, b );
}

void RenderThread::run()
{
	cout << "Render thread running" << endl;

	pcl::visualization::PCLVisualizer v;
	v.setBackgroundColor (0, 0, 0);
	//v.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	//v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	v.addCoordinateSystem (0.1);
	v.initCameraParameters ();
	
	int prevNumObj = 0;

	pcl::ModelCoefficients coefficients;
	coefficients.values.push_back(1); // Tx
	coefficients.values.push_back(.5); // Ty
	coefficients.values.push_back(0); // Tz
	coefficients.values.push_back(0); // Qx
	coefficients.values.push_back(0); // Qy
	coefficients.values.push_back(0); // Qz
	coefficients.values.push_back(0); // Qw
	coefficients.values.push_back(1); // width
	coefficients.values.push_back(.0001); // height
	coefficients.values.push_back(1.5); // depth
	//v.addCube(coefficients, "box", 0);

	while (!v.wasStopped ())
	{
		bool renderResult = false;
		results->mtx.lock();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
			if (results->getScene() ) {
				cloudCopy = results->getScene();
				renderResult = true;
			}

			boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > objects = results->getObjects();
		
		results->mtx.unlock();

		if(renderResult) {
			if(!v.updatePointCloud(cloudCopy, "scene")) {
				v.addPointCloud(cloudCopy, "scene");
				v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");
				v.resetCameraViewpoint("scene");
			}

			// Clear previous BBs and texts:
			for(int i=0; i < prevNumObj; i++) {
				// Remove bounding boxes
				std::string objName = "box" + boost::lexical_cast<std::string>(i);
				v.removePointCloud(objName);

				// Remove class texts
				std::string textName = "text" + boost::lexical_cast<std::string>(i);
				v.removeText3D(textName);
			}

			//cout << "Num objects: " << objects->size() << endl;
			for(int i=0; i < objects->size(); i++) {

				// Get bounding box coeffs
				coefficients = objects->at(i)->getBoundingBox3D();
				//pcl::ModelCoefficients* m = new pcl::ModelCoefficients(coefficients);

				// Add bounding box
				std::string objName = "box" + boost::lexical_cast<std::string>(i);
				//float w = std::max(coefficients.values.at(7), std::max( coefficients.values.at(8), coefficients.values.at(9)));
				//pcl::PointXYZ center(coefficients.values.at(0),coefficients.values.at(1),coefficients.values.at(2));
				//v.addSphere(center, w,.2,.2,.2, objName);
				//v.addCube(coefficients, objName, 0);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr box( new pcl::PointCloud<pcl::PointXYZRGB>());
				addBox(box, coefficients);
				v.addPointCloud(box, objName);
				v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, objName);

				// Add class text
				std::string textName = "text" + boost::lexical_cast<std::string>(i);
				std::string className = classNames.at( objects->at(i)->getClassification() );
				pcl::PointXYZ textLoc( coefficients.values[0] - className.length() * 0.02, coefficients.values[1] - coefficients.values[8] * 0.5 - 0.15, coefficients.values[2]);
				v.addText3D(className, textLoc, 0.05,  1,1,1,  textName);

				//cout << "Adding " << textName << endl;
			}
		}
		
		
		
		prevNumObj = objects->size();

		v.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (1000));
//		boost::thread::yield();
	}
}

