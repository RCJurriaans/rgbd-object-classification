// Main file of datagen.

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <StdAfx.h>

using namespace xn;
using namespace std;

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../SamplesConfig.xml"

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_IMAGE

#define MAX_DEPTH 10000

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
float g_pDepthHist[MAX_DEPTH];
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

unsigned int g_nViewState = DEFAULT_DISPLAY_MODE;

Context g_context;
ScriptNode g_scriptNode;
DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;

bool g_spacePressed = false;
unsigned int g_numSaves = 0;

string outFolder = ".\\saves\\";

IplImage *g_outFrameRGB;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud;
vector<XnDepthPixel*> depthHistory;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

void glutIdle (void)
{
	// Display the frame
	glutPostRedisplay();
}

void saveBuffers(const XnDepthPixel* pDepth, const XnUInt8* pImage)
{
	// The frame number
	char number[10] = "";
	sprintf(number, "%d", g_numSaves);

	// Save the RGB image
	stringstream  rgb_filename;
	rgb_filename << outFolder << "rgb_" << number << ".bmp";
	//memcpy(g_outFrameRGB->imageData, pImage, sizeof(XnUInt8) * 3 * g_imageMD.XRes() * g_imageMD.YRes());
	//cvCvtColor( g_outFrameRGB, g_outFrameRGB, CV_BGR2RGB );
	//if(!cvSaveImage(rgb_filename.str().c_str(), g_outFrameRGB)) printf("Could not save: %s\n", rgb_filename.str().c_str());

	openni_wrapper::ImageBayerGRBG imageBayer(&g_imageMD, openni_wrapper::ImageBayerGRBG::DebayeringMethod(2));// 1 == edge aware
	cv::Mat colorImage = cv::Mat::zeros (480, 640, CV_8UC3); 
	unsigned char* rgb_buffer = (unsigned char*)(colorImage.data ); 
	imageBayer.fillRGB(colorImage.cols, colorImage.rows, rgb_buffer, colorImage.step); 
	cv::cvtColor(colorImage,colorImage,CV_RGB2BGR);  // don't forget openCV uses BGR color order instead of RGB
	if(!cv::imwrite(rgb_filename.str(), colorImage)) printf("Could not save: %s\n", rgb_filename.str().c_str());

	// Save depth image
	stringstream depth_filename;
	depth_filename << outFolder << "depth_" << number << ".pld";
	unsigned int i = 0;
	for (XnUInt y = 0; y < g_depthMD.YRes(); y++)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); i++, x++, ++pDepth)
		{
			g_cloud->points[i].x = x;
			g_cloud->points[i].y = y;
			g_cloud->points[i].z = *pDepth;
		}
	}

	//pcl::io::savePCDFileASCII(depth_filename.str(), *g_cloud);
	pcl::io::savePCDFile(depth_filename.str(), *g_cloud, true);
	cout << "Saved depth and RGB data. Index: " << g_numSaves << std::endl;

	// Start cloud viewer
	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(g_cloud);
    //viewer.showCloud(cloud);
    //while (!viewer.wasStopped ())
    //{
		//Sleep(100);
    //}

	/* Old csv writeout
	ofstream depthFile;
	stringstream  filename_depth;
	filename_depth << ".\\saves\\depth_" << number << ".txt";
	depthFile.open(filename_depth.str());
	for (XnUInt y = 0; y < g_depthMD.YRes(); y++)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); x++, ++pDepth)
		{
			depthFile << *pDepth;
			if (x < g_depthMD.XRes() - 1 || y < g_depthMD.YRes() - 1)
			{
				depthFile << ",";
			}
		}
		depthFile << "\n";
	}
	depthFile.close();
	*/	
}

void glutDisplay (void)
{
	XnStatus rc = XN_STATUS_OK;

	// Read a new frame
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return;
	}
	

	//g_depth.GetMetaData(g_depthMD);
	//g_image.GetMetaData(g_imageMD);

	//g_image.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT );
	
	//openni_wrapper::ImageBayerGRBG imageBayer(&g_imageMD, openni_wrapper::ImageBayerGRBG::DebayeringMethod(2));// 1 == edge aware
	//cv::Mat colorImage = cv::Mat::zeros (480, 640, CV_8UC3); 
	//unsigned char* rgb_buffer = (unsigned char*)(colorImage.data ); 
	//imageBayer.fillRGB(colorImage.cols, colorImage.rows, rgb_buffer, colorImage.step); 
	//cv::cvtColor(colorImage,colorImage,CV_RGB2BGR);  // don't forget openCV uses BGR color order instead of RGB
	//const XnUInt8* pImage = rgb_buffer;

	//const XnUInt8* pImage = g_imageMD.Data();
	//const XnDepthPixel* pDepth = g_depthMD.Data();

	g_image.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_16_BIT );

	if (g_spacePressed)
	{
		g_spacePressed = false;
		saveBuffers(pDepth, pImage);
		g_numSaves++;
	}

	unsigned int nImageScale = GL_WIN_SIZE_X / g_depthMD.FullXRes();

	// Copied from SimpleViewer
	// Clear the OpenGL buffers
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

	// Calculate the accumulative histogram (the yellow display...)
	/*xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));

	unsigned int nNumberOfPoints = 0;
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				g_pDepthHist[*pDepth]++;
				nNumberOfPoints++;
			}
		}
	}
	for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}
	*/
	xnOSMemSet(g_pTexMap, 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));

	// check if we need to draw image frame to texture
	if (g_nViewState == DISPLAY_MODE_OVERLAY ||
		g_nViewState == DISPLAY_MODE_IMAGE)
	{
		const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
		XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

		for (XnUInt y = 0; y < g_imageMD.YRes(); ++y)
		{
			const XnRGB24Pixel* pImage = pImageRow;
			XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

			for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex)
			{
				*pTex = *pImage;
			}

			pImageRow += g_imageMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// check if we need to draw depth frame to texture
	if (g_nViewState == DISPLAY_MODE_OVERLAY ||
		g_nViewState == DISPLAY_MODE_DEPTH)
	{
		const XnDepthPixel* pDepthRow = g_depthMD.Data();
		XnRGB24Pixel* pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

		for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
		{
			const XnDepthPixel* pDepth = pDepthRow;
			XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

			for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth, ++pTex)
			{
				if (*pDepth != 0)
				{
					//int nHistValue = g_pDepthHist[*pDepth];
					int nHistValue = 255 - (*pDepth * 255) / 10000;
					pTex->nRed = nHistValue;
					pTex->nGreen = nHistValue;
					pTex->nBlue = 0;
				}
			}

			pDepthRow += g_depthMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// Create the OpenGL texture map
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glBegin(GL_QUADS);

	int nXRes = g_depthMD.FullXRes();
	int nYRes = g_depthMD.FullYRes();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)nXRes/(float)g_nTexMapX, (float)nYRes/(float)g_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)nYRes/(float)g_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();

	// Swap the OpenGL display buffers
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	XnStatus rc;
	switch (key)
	{
		case 27:
			exit (1);
		case '1':
			g_nViewState = DISPLAY_MODE_OVERLAY;
			if (g_depth.IsCapabilitySupported("AlternativeViewPoint"))
			{
				rc = g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image); 
				if (rc != XN_STATUS_OK)
				{
					printf("Failed to set viewpoint location");
				}
			}
			else
			{
				printf("Alternative Viewpoint capability not supported.");
			} 
			break;
		case '2':
			g_nViewState = DISPLAY_MODE_DEPTH;
			//g_depth.GetAlternativeViewPointCap().ResetViewPoint();
			break;
		case '3':
			g_nViewState = DISPLAY_MODE_IMAGE;
			//g_depth.GetAlternativeViewPointCap().ResetViewPoint();
			break;
		case 'm':
			g_context.SetGlobalMirror(!g_context.GetGlobalMirror());
			break;
		case 32:
			g_spacePressed = true;
			break;
	}
}


#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


int main(int argc, char* argv[])
{
	// Get start index
	//cout << "Please enter the frame-index to begin with." << endl;
	//cin >> g_numSaves;
	cout << "Initializing.." << endl;

	// Create the image that stores the frame (RGB)
	g_outFrameRGB = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	
	// Initialize the point cloud for storing the depth of the frame:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);
	g_cloud = cloud;
	cloud->width    = 640;
	cloud->height   = 480;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);

	// Create maps for storing the depth history
	for (int i = 0; i < 8; i++)
		depthHistory.push_back(new XnDepthPixel[640*480]);
	
	// Initialize OpenNI
	XnStatus rc;
	EnumerationErrors errors;
	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	// Find existing nodes (they where initialized from the xml)
	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	if (rc != XN_STATUS_OK) {
		printf("No depth node exists! Check your XML.");
		return 1;
	}
	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	if (rc != XN_STATUS_OK) {
		printf("No image node exists! Check your XML.");
		return 1;
	}

	// Align the rgb and depth data
	if (g_depth.IsCapabilitySupported("AlternativeViewPoint"))
	{
		rc = g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image); 
		if (rc != XN_STATUS_OK)
		{
			printf("Failed to set viewpoint location");
			return 1;
		}
	}
	else
	{
		printf("Alternative Viewpoint capability not supported.");
		return 1;
	}

	
	//g_image.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT ); //imageGen == xn::ImageGenerator
	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);

	// Hybrid mode isn't supported in this sample
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return 1;
	}

	// RGB is the only image format supported.
	//if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	//{
	//	printf("The device image format must be RGB24\n");
	//	return 1;
	//}

	// Texture map init
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes()-1) / 512) + 1) * 512;
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes()-1) / 512) + 1) * 512;
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

	// Debayering
	g_image.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_16_BIT ); //imageGen == xn::ImageGenerator
	//g_image.GetMetaData(imageMD); // imageMD == xn::ImageMetaData
	

	// OpenGL init
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	//glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutInitWindowSize(640, 480 );
	glutInitWindowPosition(640, 0 );
	glutCreateWindow ("Kinect Data Collector");
	//glutFullScreen();
	
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// Per frame code is in glutDisplay
	cout << "Ready. Press space to save a screenshot." << endl;
	glutMainLoop();
	
	cvReleaseImage(&g_outFrameRGB);
	for (int i=0; i < 8; i++)
		delete depthHistory[i];
	//delete &g_cloud;

	return 0;
}