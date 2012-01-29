
#include "stdafx.h"
#include "RenderThread.h"

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	cout <<"keyboard cb: " << (int)e.getKeyCode() << endl;
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
	r->shiftPressed = e.isShiftPressed();
	
	//boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	//if(e.keyDown()) {
	//	(*processInput)(e.getKeyCode());
	//}
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
				v.removeShape(objName);

				// Remove class texts
				std::string textName = "text" + boost::lexical_cast<std::string>(i);
				v.removeText3D(textName);

				//cout << "Removing " << textName << endl;
			}

			cout << "Num objects: " << objects->size() << endl;
			for(int i=0; i < objects->size(); i++) {

				// Get bounding box coeffs
				coefficients = objects->at(i)->getBoundingBox3D();
				pcl::ModelCoefficients m = coefficients;

				// Add bounding box
				float w = std::max(coefficients.values.at(7), std::max( coefficients.values.at(8), coefficients.values.at(9)));
				pcl::PointXYZ center(coefficients.values.at(0),coefficients.values.at(1),coefficients.values.at(2));
				std::string objName = "box" + boost::lexical_cast<std::string>(i);
				v.addSphere(center, w,.5,.5,.5, objName);
				//v.addCube(m, objName);
				//pcl::PointXYZ pt2(coefficients.values.at(0) + .5f,coefficients.values.at(1)+ .5f,coefficients.values.at(2));
				//v.addLine(center, pt2, objName);

				// Add class text
				pcl::PointXYZ textLoc( coefficients.values.at(0), coefficients.values.at(1) + coefficients.values.at(8) * 0.5f, coefficients.values.at(2));
				std::string textName = "text" + boost::lexical_cast<std::string>(i);
				std::string className = "class " + boost::lexical_cast<std::string>(objects->at(i)->getClassification());
				v.addText3D(className, textLoc, 0.05,  1,1,1,  textName);

				cout << "Adding " << textName << endl;
			}
		}
		
		
		
		prevNumObj = objects->size();

		v.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//		boost::thread::yield();
	}
}

