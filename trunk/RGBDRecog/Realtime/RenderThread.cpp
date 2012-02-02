
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
	int numpts = sqrt(dx*dx+dy*dy+dz*dz) * 400;
	for( int i = 0; i < numpts; i++ ) {

		if( (i / 10) % 2 == 0 ) {
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
}

void addBox2( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::ModelCoefficients& m, int r = 255, int g = 255, int b = 255)
{
	//        1 ----- 2
	//      / |      /|
	//     0 ----- 3  |
	//	   |  5 ---|- 6  
	//     |/      | /
	//     4 ------7
	addLine( cloud, m.values[0*3 + 0], m.values[0*3 + 1], m.values[0*3 + 2],
					m.values[1*3 + 0], m.values[1*3 + 1], m.values[1*3 + 2], r,g,b);//0-1
	addLine( cloud, m.values[1*3 + 0], m.values[1*3 + 1], m.values[1*3 + 2],
					m.values[2*3 + 0], m.values[2*3 + 1], m.values[2*3 + 2], r,g,b);//1-2
	addLine( cloud, m.values[2*3 + 0], m.values[2*3 + 1], m.values[2*3 + 2],
					m.values[3*3 + 0], m.values[3*3 + 1], m.values[3*3 + 2], r,g,b);//2-3
	addLine( cloud, m.values[3*3 + 0], m.values[3*3 + 1], m.values[3*3 + 2],
					m.values[0*3 + 0], m.values[0*3 + 1], m.values[0*3 + 2], r,g,b);//3-0
	addLine( cloud, m.values[0*3 + 0], m.values[0*3 + 1], m.values[0*3 + 2],
					m.values[4*3 + 0], m.values[4*3 + 1], m.values[4*3 + 2], r,g,b);//0-4
	addLine( cloud, m.values[1*3 + 0], m.values[1*3 + 1], m.values[1*3 + 2],
					m.values[5*3 + 0], m.values[5*3 + 1], m.values[5*3 + 2], r,g,b);//1-5
	addLine( cloud, m.values[2*3 + 0], m.values[2*3 + 1], m.values[2*3 + 2],
					m.values[6*3 + 0], m.values[6*3 + 1], m.values[6*3 + 2], r,g,b);//2-6
	addLine( cloud, m.values[3*3 + 0], m.values[3*3 + 1], m.values[3*3 + 2],
					m.values[7*3 + 0], m.values[7*3 + 1], m.values[7*3 + 2], r,g,b);//3-7
	addLine( cloud, m.values[4*3 + 0], m.values[4*3 + 1], m.values[4*3 + 2],
					m.values[5*3 + 0], m.values[5*3 + 1], m.values[5*3 + 2], r,g,b);//4-5
	addLine( cloud, m.values[5*3 + 0], m.values[5*3 + 1], m.values[5*3 + 2],
					m.values[6*3 + 0], m.values[6*3 + 1], m.values[6*3 + 2], r,g,b);//5-6
	addLine( cloud, m.values[6*3 + 0], m.values[6*3 + 1], m.values[6*3 + 2],
					m.values[7*3 + 0], m.values[7*3 + 1], m.values[7*3 + 2], r,g,b);//6-7
	addLine( cloud, m.values[7*3 + 0], m.values[7*3 + 1], m.values[7*3 + 2],
					m.values[4*3 + 0], m.values[4*3 + 1], m.values[4*3 + 2], r,g,b);//7-4
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
	v.setBackgroundColor (0, 0.08, 0.15);
	//v.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	//v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	v.addCoordinateSystem (0.1);
	v.initCameraParameters ();
	
	int prevNumObj = 0;

	pcl::ModelCoefficients coefficients;

	while (!v.wasStopped ())
	{
		bool renderResult = false;
		bool renderBox = false;
		bool changeColors = false;
		bool classify = false;

		// Get the input from classifier
		results->mtx.lock();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
			if (results->getScene() ) {
				cloudCopy = results->getSceneFiltered();
				renderResult = true;
			}
			boost::shared_ptr<pcl::PointIndices> objectInliers = results->getObjectInliers();
			boost::shared_ptr<pcl::PointIndices> planeInliers = results->getPlaneInliers();
			boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > objects = results->getObjects();
			if( results->hasNew && results->renderPlane) {
				results->hasNew = false;
				changeColors = true;
			}
			renderBox = results->renderBox;
			classify = results->classify;

			if(results->newClassName != "") {
				cout << "RenderThread: adding classname " << results->newClassName << endl;
				classNames.push_back(results->newClassName);
				results->newClassName = "";
			}
		results->mtx.unlock();



		if(renderResult) {

			
			if( changeColors ) {

				/*for(int i=0; i < objectInliers->indices.size(); i++) {
				
					pcl::PointXYZRGB& pt = cloudCopy->at(objectInliers->indices.at(i));
					uint32_t rgbin = *reinterpret_cast<int*>(&pt.rgb);
					uint8_t r = (rgbin >> 16) & 0x0000ff;
					uint8_t g = (rgbin >> 8)  & 0x0000ff;
					uint8_t b = (rgbin)       & 0x0000ff;

					uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
									static_cast<uint32_t>(g) << 8 |
									static_cast<uint32_t>(b));
					cloudCopy->at(objectInliers->indices.at(i)).rgb = *reinterpret_cast<float*>(&rgb);
				}*/

				// Render plane inliers
				for(int i=0; i < planeInliers->indices.size(); i++) {
				
					pcl::PointXYZRGB& pt = cloudCopy->at(planeInliers->indices.at(i));
					//uint32_t rgbin = *reinterpret_cast<int*>(&pt.rgb);
					//uint8_t r = (rgbin >> 16) & 0x0000ff;
					//uint8_t g = (rgbin >> 8)  & 0x0000ff;
					//uint8_t b = (rgbin)       & 0x0000ff;

					uint32_t rgb;
					if( (int)(pt.x * 100) % 10 == 0 || (int)(pt.z * 100) % 10 == 0 ) {
						uint8_t r = 0;
						uint8_t g = 190;
						uint8_t b = 200;
						rgb = (static_cast<uint32_t>(r) << 16 |
							   static_cast<uint32_t>(g) << 8 |
							   static_cast<uint32_t>(b));
						pt.rgb = *reinterpret_cast<float*>(&rgb);
					} else {
						if( (int)(pt.x * 100) % 5 == 0 || (int)(pt.z * 100) % 5 == 0 ) {
							uint8_t r = 0;
							uint8_t g = 30;
							uint8_t b = 60;
							rgb =  (static_cast<uint32_t>(r) << 16 |
									static_cast<uint32_t>(g) << 8 |
									static_cast<uint32_t>(b));
							pt.rgb = *reinterpret_cast<float*>(&rgb);
						} else {
							uint8_t r = 0;
							uint8_t g = 51;
							uint8_t b = 86;
							rgb =  (static_cast<uint32_t>(r) << 16 |
									static_cast<uint32_t>(g) << 8 |
									static_cast<uint32_t>(b));
							pt.rgb = *reinterpret_cast<float*>(&rgb);
						}
					}
				}
			}

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

				if( renderBox) {
					// Add bounding box
					std::string objName = "box" + boost::lexical_cast<std::string>(i);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr box( new pcl::PointCloud<pcl::PointXYZRGB>());
					addBox2(box, coefficients, 0, 255, 0);
					v.addPointCloud(box, objName);
					v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, objName);
				}

				if( classify) {
					// Add class text
					std::string className;
					if( objects->at(i)->getClassification() != -1 ) {
						className = classNames.at( objects->at(i)->getClassification() );
					} else {
						className = "omgwtfbbq";
					}
					std::string textName = "text" + boost::lexical_cast<std::string>(i);
					float centerx = (coefficients.values[0*3+0] + coefficients.values[1*3+0] + coefficients.values[2*3+0] + coefficients.values[3*3+0] + coefficients.values[4*3+0] + coefficients.values[5*3+0] + coefficients.values[6*3+0] + coefficients.values[7*3+0]) / 8;
					float centery = (coefficients.values[0*3+1] + coefficients.values[1*3+1] + coefficients.values[2*3+1] + coefficients.values[3*3+1] + coefficients.values[4*3+1] + coefficients.values[5*3+1] + coefficients.values[6*3+1] + coefficients.values[7*3+1]) / 8;
					float centerz = (coefficients.values[0*3+2] + coefficients.values[1*3+2] + coefficients.values[2*3+2] + coefficients.values[3*3+2] + coefficients.values[4*3+2] + coefficients.values[5*3+2] + coefficients.values[6*3+2] + coefficients.values[7*3+2]) / 8;
					float miny = min(coefficients.values[0*3+1], min(coefficients.values[1*3+1], min(coefficients.values[2*3+1], min(coefficients.values[3*3+1], min(coefficients.values[4*3+1],min(coefficients.values[5*3+1],min(coefficients.values[6*3+1],coefficients.values[7*3+1])))))));
					pcl::PointXYZ textLoc(centerx, miny - 0.1, centerz);
					v.addText3D(className, textLoc, 0.04,  .8,1,.9,  textName);
				}
			}
		}
		
		prevNumObj = objects->size();

		v.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//		boost::thread::yield();
	}
}

