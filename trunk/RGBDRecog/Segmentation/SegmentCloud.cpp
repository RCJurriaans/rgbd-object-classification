#include "stdafx.h"
#include "SegmentCloud.h"


// Blobb stuff
#include "ImageAccess.h"

#include <pcl/io/pcd_io.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>

#include <pcl/range_image/range_image.h>

boost::shared_ptr<cv::Mat> SegmentCloud::getMask(	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background, boost::shared_ptr<pcl::PointIndices> inliers)
{
	int nmax;

	boost::shared_ptr<cv::Mat> mask(new cv::Mat( input->height, input->width, CV_8UC1 ) );

	float point_backz;
	float point_imgz;

	nmax = input->width * input->height;
	for(int n=0; n < nmax ; n++ ){
		// Extract z value from both clouds
		point_backz = background->at(n).z;
		point_imgz  = input->at(n).z;

		// Recalculate indices in matrix from n
		int i = n % input->width;
		int j = ((n-i) / input->width);

		// Check whether the distance is smaller than a threshold
		// Check whether the distance is within the range of the RGB-D camera
		// Check for QNaN
		if(abs(point_backz-point_imgz)<threshold || point_imgz>maxDistanceFilter || point_imgz<minDistanceFilter || point_imgz != point_imgz){
			//input->points[n].z = std::numeric_limits<float>::quiet_NaN();
			mask->data[j*mask->step[0]+i*mask->step[1]] = 0;
		}
		else {
			mask->data[j*mask->step[0]+i*mask->step[1]] = 255;
			inliers->indices.push_back(n);
		}
	}

	return mask;
}

boost::shared_ptr<cv::Mat>
	SegmentCloud::getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background)
{
	boost::shared_ptr<pcl::PointIndices> inliers(new pcl::PointIndices);
	return getMask(input, background, inliers);
}

boost::shared_ptr<cv::Mat>
	SegmentCloud::getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<pcl::PointIndices> inliers)
{
	switch( crtMethod ) {
	case SegBack:
		return getMask(input, this->background, inliers);
		break;
	case SegObj:
		break;
	case SegPlane:
		return getPlaneMask(input, inliers);
		break;
	case SegNormHist:
		break;
	default:
		std::cerr << "Invallid segmentation method" << std::endl;
		return boost::shared_ptr<cv::Mat>();
	}

}

boost::shared_ptr<cv::Mat>
	SegmentCloud::getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
{
	boost::shared_ptr<pcl::PointIndices> inliers(new pcl::PointIndices);
	return getMask(input, inliers);
}

boost::shared_ptr<cv::Mat>
	SegmentCloud::getPlaneMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,  boost::shared_ptr<pcl::PointIndices> objectinliers)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Use pass through filter to extract too far points.
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (input);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (minDistanceFilter, maxDistanceFilter);
	pass.setKeepOrganized(true);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	boost::shared_ptr<cv::Mat> mask(new cv::Mat( cloud_filtered->height, cloud_filtered->width, CV_8UC1 ) );

	// Fit plane and extract points
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	// Optional (increases performance)
	seg.setOptimizeCoefficients (true);

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMaxIterations(10);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);

	int nmax = cloud_filtered->size();
	int i;
	int j;
	int inliercount = 0;
	for(int n= 0; n<nmax; n++){
		// Recalculate indices in matrix from n
		i = n % input->width;
		j = ((n-i) / input->width);
		if( inliers->indices[inliercount]==n){
			mask->data[j*mask->step[0]+i*mask->step[1]] = 0;
			inliercount++;
		}
		else{
			mask->data[j*mask->step[0]+i*mask->step[1]] = 255;
			objectinliers->indices.push_back(n);
		}
	}

	return mask;
	//return cloud_p;
}


boost::shared_ptr<std::vector<cv::Rect> > SegmentCloud::getROIS(boost::shared_ptr<const cv::Mat> mask)
{
	boost::shared_ptr<std::vector<cv::Rect> > rois(new std::vector<cv::Rect>);
	IplImage ipl_bmask = *mask;//cvCreateImage(cvSize(BooleanMask.size().height,BooleanMask.size().width),8,1);
	//std::cout << "Created ipl version of mask " <<  std::endl;
	cvSetImageROI(&ipl_bmask, cvRect(0,0,ipl_bmask.width, ipl_bmask.height));
	//BwImage enter(ipl_bmask);
	imcalc.Calculate(&ipl_bmask, 25);
		
	int minx;
	int miny;
	int mean_x;
	int mean_y;
	int boxWidth;
	int boxHeight;
	int rc;

	for(rc = 0; rc<25 ; rc++){


		mean_x = imcalc.getmean(rc,0);
		if(mean_x==0){break;}

		mean_y = imcalc.getmean(rc,1);

		boxWidth  = imcalc.getRegions(rc,0);
		boxHeight = imcalc.getRegions(rc,1);

		boxWidth = static_cast<int>(boxWidth*1.5);
		boxHeight = static_cast<int>(boxHeight*1.5);

		minx = mean_x-(boxWidth/2);
		miny = mean_y-(boxHeight/2); 

		if(minx<0){
			boxWidth -= minx;
			minx = 0;
		}
		if(miny<0){
			boxHeight -= miny;
			miny = 0;
		}

		//fixed these errors checks //Tijmen
		if(minx+boxWidth>ipl_bmask.width){
			boxWidth = (ipl_bmask.width-minx)-1;
		}
		if(miny+boxHeight>ipl_bmask.height){
			boxHeight = (ipl_bmask.height-miny)-1;
		}

		rois->push_back(cv::Rect(minx, miny, boxWidth, boxHeight));
	}

	std::cout << "Found " << rc << " regions" << std::endl;
	return rois;
}

cv::Rect SegmentCloud::getROI(boost::shared_ptr<const cv::Mat> mask)
{
	IplImage ipl_bmask = *mask;//cvCreateImage(cvSize(BooleanMask.size().height,BooleanMask.size().width),8,1);
	//std::cout << "Created ipl version of mask " <<  std::endl;
	cvSetImageROI(&ipl_bmask, cvRect(0,0,ipl_bmask.width, ipl_bmask.height));
	//BwImage enter(ipl_bmask);
	imcalc.Calculate(&ipl_bmask, 1);

	int mean_x = imcalc.getmean(0,0);
	int mean_y = imcalc.getmean(0,1);

	int boxWidth  = imcalc.getRegions(0,0);
	int boxHeight = imcalc.getRegions(0,1);

	boxWidth = static_cast<int>(boxWidth*1.5);
	boxHeight = static_cast<int>(boxHeight*1.5);

	int minx;
	int miny;

	minx = mean_x-(boxWidth/2);
	miny = mean_y-(boxHeight/2); 

	if(minx<0){
		boxWidth -= minx;
		minx = 0;
	}
	if(miny<0){
		boxHeight -= miny;
		miny = 0;
	}

	//fixed these errors checks //Tijmen
	if(minx+boxWidth>ipl_bmask.width){
		boxWidth = (ipl_bmask.width-minx)-1;
	}
	if(miny+boxHeight>ipl_bmask.height){
		boxHeight = (ipl_bmask.height-miny)-1;
	}



	return cv::Rect(minx, miny, boxWidth, boxHeight);
}

	  pcl::ModelCoefficients
		  SegmentCloud::getCoefficients(cv::Rect ROI, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {
		  pcl::ModelCoefficients coeff;

		  float minx=0, miny=0, minz=0;
		  float maxx=0, maxy=0, maxz=0;
		  float avgx=0, avgy=0, avgz=0;

		  int imin = ROI.x;
		  int imax = imin+ROI.width;
		  int jmin = ROI.y;
		  int jmax = jmin+ROI.height;
		  
		  int pointcount=0;


		  pcl::PointXYZRGB crtPoint;
		  for(int i=imin; i<imax; i++)
		  {
			  for(int j=jmin; j<jmax; j++)
			  {
				  crtPoint = input->at(i,j);
				  if(crtPoint.x<minx){minx=crtPoint.x;}
				  if(crtPoint.y<miny){miny=crtPoint.y;}
				  if(crtPoint.z<minz){minz=crtPoint.z;}

				  if(crtPoint.x>maxx){maxx=crtPoint.x;}
				  if(crtPoint.y>maxy){maxy=crtPoint.y;}
				  if(crtPoint.z>maxz){maxz=crtPoint.z;}

				  if(crtPoint.x==crtPoint.x && crtPoint.y==crtPoint.y && crtPoint.z==crtPoint.z){
				  avgx+=crtPoint.x;
				  avgy+=crtPoint.y;
				  avgz+=crtPoint.z;
				  pointcount++;
				  }

			  }
		  }

		  coeff.values.push_back(avgx / pointcount); // Tx
		  coeff.values.push_back(avgy / pointcount); // Ty
		  coeff.values.push_back(avgz / pointcount); // Tz
		  coeff.values.push_back(0); // Qx
		  coeff.values.push_back(0); // Qy
		  coeff.values.push_back(0); // Qz
		  coeff.values.push_back(0); // Qw
		  coeff.values.push_back(maxx-minx); // width
		  coeff.values.push_back(maxy-miny); // height
		  coeff.values.push_back(maxz-minz); // depth

		  std::cout << coeff << std::endl;

		  return coeff;
	  }

	  boost::shared_ptr<std::vector<pcl::ModelCoefficients> >
		  SegmentCloud::getCoefficients(boost::shared_ptr<std::vector<cv::Rect> > ROIS, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {
		  boost::shared_ptr<std::vector<pcl::ModelCoefficients> > coeffs;
		  for(int i=0; i<ROIS->size() ; i++)
		  {
			  coeffs->push_back(getCoefficients(ROIS->at(i), input));
		  }
		  return coeffs;
	  }


pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentCloud::getWindowCloud(const cv::Rect& ROI,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
{
	// We don't want to use imcalc here, because it makes the behaviour of SegmentCloud dependent on the order of the calls
	//int mean_x = imcalc.getmean(0,0);
	//int mean_y = imcalc.getmean(0,1);
	int mean_x = ROI.x + ROI.width / 2;
	int mean_y = ROI.y + ROI.height / 2;

	//int boxWidth  = static_cast<int>(imcalc.getRegions(0,0)*1.2);
	//int boxHeight = static_cast<int>(imcalc.getRegions(0,1)*1.2);
	int boxWidth = ROI.width;
	int boxHeight = ROI.height;

	int jmin = mean_x-boxWidth/2;
	int jmax = mean_x+boxWidth/2-1;
	int imin = mean_y-boxHeight/2;
	int imax = mean_y+boxHeight/2-1;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr windowCloud  (new pcl::PointCloud<pcl::PointXYZRGB>);

	windowCloud->width = boxWidth;
	windowCloud->height = boxHeight;
	windowCloud->points.resize (windowCloud->width * windowCloud->height);

	int nmax = windowCloud->width * windowCloud->height;

	int i = imin;
	int j = jmin;

	for(int n=0; n < nmax ; n++ ){
		windowCloud->points[n] = input->at(j,i);
		if(i==imax){
			j++;
			i=imin;
		}
		else
		{
			i++;
		}
	}

	return windowCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	SegmentCloud::getUnorgCloud(boost::shared_ptr<const cv::Mat> mask,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
{
	int nmax = input->width * input->height;
	//mask.release();
	//BooleanMask = cv::Mat::ones( inputCloud->height, inputCloud->width, CV_8UC1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(int n=0; n < nmax ; n++ ){
		int i = n % input->width;
		int j = ((n-i) / input->width);

		if( mask->data[j*mask->step[0]+i] )
			segmentCloud->push_back(input->at(n));
	}
	return segmentCloud;
	/* doesn't depend on seg method.
	switch( getSegMethod() ){
	case SegBack:

	float point_backz;
	float point_imgz;

	nmax = inputCloud->width * inputCloud->height;
	std::cout << "Created segment object" << std::endl;
	for(int n=0; n < nmax ; n++ ){
	// Extract z value from both clouds
	point_backz = backgroundCloud->at(n).z;
	point_imgz  = inputCloud->at(n).z;

	// Recalculate indices in matrix from n
	int i = n % inputCloud->width;
	int j = ((n-i) / inputCloud->width);

	// Check whether the distance is smaller than a threshold
	// Check whether the distance is within the range of the RGB-D camera
	// Check for QNaN
	if(abs(point_backz-point_imgz)<threshold || point_imgz>maxDistanceFilter || point_imgz<minDistanceFilter || point_imgz != point_imgz){
	BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 0;
	}
	else {
	segmentCloud->push_back(inputCloud->at(n));
	//BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 1;
	}
	}
	inputCloud->swap(*segmentCloud);

	break;
	case SegObj:
	std::cout << "No such method yet" << std::endl;
	break;
	case SegPlane:
	std::cout << "No such method yet" << std::endl;
	break;
	case SegNormHist:
	std::cout << "No such method yet" << std::endl;
	break;
	}
	*/
}

