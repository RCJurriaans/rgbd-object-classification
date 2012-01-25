
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

void RenderThread::visCallback(pcl::visualization::PCLVisualizer& vis)
{
	std::vector<pcl::visualization::Camera> cameras;
	vis.getCameras(cameras);
	//cameras(0).
	//cout <<"viscallback: "<<rightPressed <<endl;
	if(rightPressed) {
		cout << "focal: " << cameras[0].focal[0] << " " << cameras[0].focal[1] << " " << cameras[0].focal[2] << endl;
		cout << "pos: " <<cameras[0].pos[0] << " " << cameras[0].pos[1] << " " << cameras[0].pos[2] << endl;
		cout << "view: " << cameras[0].view[0] << " " << cameras[0].view[1] << " " << cameras[0].view[2] << endl;

		//cameras[0].focal[0] = 1;
		//vis.setCameraPosition(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2],
		//						cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
		//vis.setCameraPosition(0,0,25,  1,1,0);
		//vis.resetCameraViewpoint();
		//vis.getViewerPose
		//vis.updateCamera();
		//cout << "focal: " << cameras[0].focal[0] << " " << cameras[0].focal[1] << " " << cameras[0].focal[2] << endl;
		//cout << "pos: " <<cameras[0].pos[0] << " " << cameras[0].pos[1] << " " << cameras[0].pos[2] << endl;
		//cout << "view: " << cameras[0].view[0] << " " << cameras[0].view[1] << " " << cameras[0].view[2] << endl;

	}

	bool renderResult = false;
	results->mtx.lock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
		if (results->getScene() ) {
			cloudCopy = results->getScene();
			renderResult = true;
		}

		//results->getObjects(

	results->mtx.unlock();

	if(renderResult) {
		if(!vis.updatePointCloud(cloudCopy)) {
			vis.addPointCloud(cloudCopy);
		}

		/*pcl::ModelCoefficients coefficients;
		coefficients.values.push_back(0); // Tx
		coefficients.values.push_back(0); // Ty
		coefficients.values.push_back(0); // Tz
		coefficients.values.push_back(0); // Qx
		coefficients.values.push_back(0); // Qy
		coefficients.values.push_back(0); // Qz
		coefficients.values.push_back(0); // Qw
		coefficients.values.push_back(1); // width
		coefficients.values.push_back(.5); // height
		coefficients.values.push_back(1.5); // depth
		vis.removeAllShapes();
		vis.addCube(coefficients);*/
	}
}

void RenderThread::run()
{
	cout << "Render thread running" << endl;
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
	//cout << "renderthread loading file: " << pcl::io::loadPCDFile<pcl::PointXYZRGB> ("img001.pcd", *cloud_back) <<endl;
	//cloud = cloud_back;

	viewer = new pcl::visualization::CloudViewer("VIEWER");
	//viewer->showCloud(cloud);
	
	// Register visualization callback that does the rendering
	boost::function1<void, pcl::visualization::PCLVisualizer&> f = boost::bind(&RenderThread::visCallback, this, _1);
	viewer->runOnVisualizationThread(f);

	// Register keyboard callback
	//boost::function2<void, const pcl::visualization::KeyboardEvent&, void*> f2 = boost::bind(&RenderThread::keyboardCallback, this, _1, _2);
	viewer->registerKeyboardCallback( &keyboardCB, this );
	//viewer->registerKeyboardCallback( f2, NULL );


	bool renderResult = false;
	while(true)
	{
		/*
		results->mtx.lock();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
			
			//if( results->getObjects()->size() > 0 )
			//{
			//	
			//	if (results->getObjects()->at(0)->cloud) {
			//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr toCopy = results->getObjects()->at(0)->cloud;
			//		//pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*toCopy, *cloudCopy);
			//		cloudCopy = toCopy;
			//		renderResult=true;

			//	}
			//	else
			//		renderResult=false;
			//}
			//else
			//	renderResult=false;
			
			
			if (results->getScene() ) {
				cloudCopy = results->getScene();
				renderResult = true;
			}
			else{
				renderResult = false;
			}

		results->mtx.unlock();

		if (renderResult) {
			cout << "rendering copy" << endl;
			viewer->showCloud(cloudCopy);
		}
		else {
			//cout << "rendering scene"<<endl;
			//viewer->showCloud(cloud);
		}*/
		
		boost::thread::yield();
	}
}

