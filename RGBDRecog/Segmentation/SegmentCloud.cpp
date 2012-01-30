#include "stdafx.h"
#include "SegmentCloud.h"


// Blobb stuff
#include "ImageAccess.h"

#include "gdiam.h"


	//        1 ----- 2
	//      / |      /|
	//     0 ----- 3  |
	//	   |  5 ---|- 6  
	//     |/      | /
	//     4 ------7
	// 0.x 0.y 0.z 1.x 1.y 1.z 2.x 2.y ...
pcl::ModelCoefficients SegmentCloud::getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input){
	pcl::ModelCoefficients coeffs;
	
	gdiam_real  * points;
	int num=input->size();

	points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * num );
    assert( points != NULL );

	// Initialize points in vector
	for  ( int  ind = 0; ind < num; ind++ ) {
        points[ ind * 3 + 0 ] = input->at(ind).x;
        points[ ind * 3 + 1 ] = input->at(ind).y;
        points[ ind * 3 + 2 ] = input->at(ind).z;
    }

	GPointPair   pair;

    printf( "Computing the diameter for %d points selected "
            "uniformly from the unit cube\n", num );
    pair = gdiam_approx_diam_pair( (gdiam_real *)points, num, 0.0 );
    printf( "Diameter distance: %g\n", pair.distance );
    printf( "Points realizing the diameter\n"
            "\t(%g, %g, %g) - (%g, %g, %g)\n",
            pair.p[ 0 ], pair.p[ 1 ], pair.p[ 2 ],
            pair.q[ 0 ], pair.q[ 1 ], pair.q[ 2 ] );

    
    gdiam_point  * pnt_arr;
    gdiam_bbox   bb;

    pnt_arr = gdiam_convert( (gdiam_real *)points, num );

    printf( "Computing a tight-fitting bounding box of the point-set\n" );
    bb = gdiam_approx_mvbb_grid_sample( pnt_arr, num, 5, 400 );

    printf( "Resulting bounding box:\n" );
    bb.dump();


	return coeffs;

}


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
	pass.setFilterLimits (minDistanceFilter, 2);
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
	//seg.setOptimizeCoefficients (true);

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMaxIterations(10);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);
	if(inliers->indices.size()==0){
		return mask;
	}

	int nmax = cloud_filtered->size();
	int i;
	int j;
	int inliercount=0;
	for(int n= 0; n<nmax; n++)
	{
		float imgz = cloud_filtered->at(n).z;
		// Recalculate indices in matrix from n
		i = n % input->width;
		j = ((n-i) / input->width);
		mask->data[j*mask->step[0]+i*mask->step[1]] = 0;

		if(!(inliercount>=inliers->indices.size() || inliers->indices.at(inliercount)==n || (imgz!=imgz))){
			mask->data[j*mask->step[0]+i*mask->step[1]] = 255;
			objectinliers->indices.push_back(n);
		}
		else{
			
			if(inliercount<inliers->indices.size() && inliers->indices.at(inliercount)==n){inliercount++;}
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

	//std::cout << "Found " << rc << " regions" << std::endl;
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


pcl::ModelCoefficients
	SegmentCloud::getCoefficients(cv::Rect ROI, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<const cv::Mat> mask)
{
	pcl::ModelCoefficients coeff;

	double minx=0, miny=0, minz=0;
	double maxx=0, maxy=0, maxz=0;
	double avgx=0, avgy=0, avgz=0;
	double avg2x=0,avg2y=0,avg2z=0;
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
			if( mask->data[j*mask->step[0]+i] != 0 ) {
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
					avg2x+= crtPoint.x*crtPoint.x;
					avg2y+= crtPoint.y*crtPoint.y;
					avg2z+= crtPoint.z*crtPoint.z;
					pointcount++;
				}
			}
		}
	}
	avgx /= pointcount;
	avgy /= pointcount;
	avgz /= pointcount;
	avg2x /= pointcount;
	avg2y /= pointcount;
	avg2z /= pointcount;

	coeff.values.push_back(avgx); // Tx
	coeff.values.push_back(avgy); // Ty
	coeff.values.push_back(avgz); // Tz
	coeff.values.push_back(0); // Qx
	coeff.values.push_back(0); // Qy
	coeff.values.push_back(0); // Qz
	coeff.values.push_back(0); // Qw
	coeff.values.push_back(sqrt(std::max(avg2x - avgx*avgx, 0.001))*4);//maxx-minx);//(sqrt(sigmax)*3);//; // width
	coeff.values.push_back(sqrt(std::max(avg2y - avgy*avgy, 0.001))*4);//maxy-miny);//(sqrt(sigmay)*3);//maxy-miny); // height
	coeff.values.push_back(sqrt(std::max(avg2z - avgz*avgz, 0.001))*4);//maxz-minz);//(sqrt(sigmaz)*3);//maxz-minz); // depth
	//coeff.values.push_back(maxx-minx); // width
	//coeff.values.push_back(maxy-miny); // height
	//coeff.values.push_back(maxz-minz); // depth
	//coeff.values.push_back(sqrt(std::max(avg2z - avgz*avgz, 0.001))*3);//maxz-minz);//(sqrt(sigmaz)*3);//maxz-minz); // depth

	//std::cout << coeff << std::endl;
	//std::cout << sigmaz << " " << avg2z-avgz*avgz << std::endl;

	return coeff;
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

		if( mask->data[j*mask->step[0]+i] && input->at(n).x!=input->at(n).x && input->at(n).x!=input->at(n).y && input->at(n).x!=input->at(n).z)
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

boost::shared_ptr<cv::Mat> SegmentCloud::denoizeMask( boost::shared_ptr<cv::Mat> latestMask )
{
	boost::shared_ptr<cv::Mat> smoothed( new cv::Mat(latestMask->clone()) );

	maskHistory.push_back( latestMask );

	static const int maxHist = 3;
	if(maskHistory.size() > maxHist) {
		maskHistory.erase(maskHistory.begin());
	}

	for( int i = 0; i < smoothed->cols; i++) {
		for( int j = 0; j < smoothed->rows; j++) {

			bool hasZero = false;
			for( int k = 0; k < maskHistory.size(); k++ ){
				if(maskHistory.at(k)->data[j * smoothed->step[0] + i] == 0)
					hasZero = true;
			}

			if(hasZero)
				smoothed->data[j * smoothed->step[0] + i] = 0;
		}
	}

	return smoothed;
}
