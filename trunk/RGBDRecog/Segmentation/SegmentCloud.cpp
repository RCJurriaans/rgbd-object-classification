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

pcl::ModelCoefficients SegmentCloud::getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr unorgCloud){

	gdiam_real* points;
	
	// Optimization
	int stride = 1;

	if(unorgCloud->size()>1100){
		stride *= std::ceil((static_cast<double>(unorgCloud->size())/1000));
	}

	int gridsize = 3;
	int samplesize = 50;


	points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * ((unorgCloud->size()+stride-1) / stride) );
	assert( points != NULL );

	pcl::PointXYZRGB pt;

	
	int ind = 0;
	for( int i = 0; i < unorgCloud->size(); i+=stride) {
		pt = unorgCloud->at(i);
		points[ ind * 3 + 0 ] = (double) pt.x;// + (((double) rand() / (RAND_MAX+1))*0.02);;
		points[ ind * 3 + 1 ] = (double) pt.y;// + (((double) rand() / (RAND_MAX+1))*0.02);;
		points[ ind * 3 + 2 ] = (double) pt.z + (((double) rand() / (RAND_MAX+1))*0.01);
		ind++;
	}

	gdiam_point* pnt_arr;
	gdiam_bbox bb;

	pnt_arr = gdiam_convert( (gdiam_real *)points, ((unorgCloud->size()+stride-1) / stride) );

	//clock_t begin, end; //for timing
	//begin=clock();
	bb = gdiam_approx_mvbb_grid_sample( pnt_arr, ((unorgCloud->size()+stride-1) / stride), gridsize, samplesize );//num, 5, 400 );
	//end=clock()-begin;
	//std::cout << "time!!!: " << (double)end / ((double)CLOCKS_PER_SEC) << std::endl;
	
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(bb.getPoint(0,0,0,0));
	coeffs.values.push_back(bb.getPoint(0,0,0,1));
	coeffs.values.push_back(bb.getPoint(0,0,0,2));
	
	coeffs.values.push_back(bb.getPoint(0,0,1,0));
	coeffs.values.push_back(bb.getPoint(0,0,1,1));
	coeffs.values.push_back(bb.getPoint(0,0,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,1,0));
	coeffs.values.push_back(bb.getPoint(0,1,1,1));
	coeffs.values.push_back(bb.getPoint(0,1,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,0,0));
	coeffs.values.push_back(bb.getPoint(0,1,0,1));
	coeffs.values.push_back(bb.getPoint(0,1,0,2));
	
	coeffs.values.push_back(bb.getPoint(1,0,0,0));
	coeffs.values.push_back(bb.getPoint(1,0,0,1));
	coeffs.values.push_back(bb.getPoint(1,0,0,2));

	coeffs.values.push_back(bb.getPoint(1,0,1,0));
	coeffs.values.push_back(bb.getPoint(1,0,1,1));
	coeffs.values.push_back(bb.getPoint(1,0,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,1,0));
	coeffs.values.push_back(bb.getPoint(1,1,1,1));
	coeffs.values.push_back(bb.getPoint(1,1,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,0,0));
	coeffs.values.push_back(bb.getPoint(1,1,0,1));
	coeffs.values.push_back(bb.getPoint(1,1,0,2));

	free(pnt_arr);
	free(points);

	return coeffs;

}
pcl::ModelCoefficients SegmentCloud::getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, cv::Mat mask, cv::Rect ROI){
	pcl::ModelCoefficients coeffs;


	float minx=0, miny=0, minz=0;
	float maxx=0, maxy=0, maxz=0;
	float avgx=0, avgy=0, avgz=0;
	float avg2x=0,avg2y=0,avg2z=0;
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
			if( mask.data[j*mask.step[0]+i] != 0 ) {
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

	/*float varx = sqrt(std::max(avg2x - avgx*avgx, 0.001))*4
	float vary = sqrt(std::max(avg2x - avgx*avgx, 0.001))*4
	float varz = sqrt(std::max(avg2x - avgx*avgx, 0.001))*4
	float minx = avgx - varx;
	float maxx = avgx + varx;
	float miny = avgy - vary;
	float maxy = avgy + vary;
	float minz = avgz - varz;
	float maxz = avgz + varz;*/


	gdiam_real  * points;
	int num=input->size();

	points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * num );
	assert( points != NULL );

	// Initialize points in vector
	for  ( int  ind = 0; ind < num; ind++ ) {
		
		points[ ind * 3 + 0 ] = (double) input->at(ind).x;
		points[ ind * 3 + 1 ] = (double) input->at(ind).y;
		points[ ind * 3 + 2 ] = (double) input->at(ind).z;// + (((double) rand() / (RAND_MAX+1))*0.01);
		//points[ ind * 3 + 0 ] = ((double) rand() / (RAND_MAX+1));
        //points[ ind * 3 + 1 ] = ((double) rand() / (RAND_MAX+1));
        //points[ ind * 3 + 2 ] = ((double) rand() / (RAND_MAX+1));
	}



	gdiam_point  * pnt_arr;
	gdiam_bbox   bb;

	pnt_arr = gdiam_convert( (gdiam_real *)points, num );

	printf( "Computing a tight-fitting bounding box of the point-set\n" );
	bb = gdiam_approx_mvbb_grid_sample( pnt_arr, num, 5, 400 );

	//printf( "Resulting bounding box:\n" );
	
	coeffs.values.push_back(bb.getPoint(0,0,0,0));
	coeffs.values.push_back(bb.getPoint(0,0,0,1));
	coeffs.values.push_back(bb.getPoint(0,0,0,2));
	
	coeffs.values.push_back(bb.getPoint(0,0,1,0));
	coeffs.values.push_back(bb.getPoint(0,0,1,1));
	coeffs.values.push_back(bb.getPoint(0,0,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,1,0));
	coeffs.values.push_back(bb.getPoint(0,1,1,1));
	coeffs.values.push_back(bb.getPoint(0,1,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,0,0));
	coeffs.values.push_back(bb.getPoint(0,1,0,1));
	coeffs.values.push_back(bb.getPoint(0,1,0,2));
	
	coeffs.values.push_back(bb.getPoint(1,0,0,0));
	coeffs.values.push_back(bb.getPoint(1,0,0,1));
	coeffs.values.push_back(bb.getPoint(1,0,0,2));

	coeffs.values.push_back(bb.getPoint(1,0,1,0));
	coeffs.values.push_back(bb.getPoint(1,0,1,1));
	coeffs.values.push_back(bb.getPoint(1,0,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,1,0));
	coeffs.values.push_back(bb.getPoint(1,1,1,1));
	coeffs.values.push_back(bb.getPoint(1,1,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,0,0));
	coeffs.values.push_back(bb.getPoint(1,1,0,1));
	coeffs.values.push_back(bb.getPoint(1,1,0,2));

	std::cout << coeffs << std::endl;
	//bb.dump();

	return coeffs;

}/*
pcl::ModelCoefficients SegmentCloud::getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, cv::Mat mask, cv::Rect ROI){
	
	pcl::ModelCoefficients coeffs;
	float minz =1000000000, maxz=-1000000000000;
	//float minx=100000000000, miny=100000000000, minz=100000000000;
	//float maxx=-100000000000, maxy=-10000000000, maxz=-10000000000;
	//float avgx=0, avgy=0, avgz=0;
	//float avg2x=0,avg2y=0,avg2z=0;
	int imin = ROI.x;
	int imax = imin+ROI.width;
	int jmin = ROI.y;
	int jmax = jmin+ROI.height;

	int pointcount=0;

	pcl::PointXYZRGB crtPoint;
	for(int i=imin; i<imax; i+=1)//i++)
	{
		for(int j=jmin; j<jmax; j+=1)//j++)
		{
			if( mask.data[j*mask.step[0]+i] != 0 ) {
				crtPoint = input->at(i,j);

				if(crtPoint.x==crtPoint.x && crtPoint.y==crtPoint.y && crtPoint.z==crtPoint.z){
					pointcount++;
				}
			}
		}
	}
	std::cout << "Points found: " << pointcount << std::endl;

	gdiam_real* points;
	points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * pointcount );
	assert( points != NULL );

	int sind = 0;
	for(int i=imin; i<imax; i+=1)//i++)
	{
		for(int j=jmin; j<jmax; j+=1)//j++)
		{
			if( mask.data[j*mask.step[0]+i] != 0 ) {
				
				crtPoint = input->at(i,j);
				if(crtPoint.x==crtPoint.x && crtPoint.y==crtPoint.y && crtPoint.z==crtPoint.z){
					points[ sind * 3 + 0 ] = (double) crtPoint.x;
					points[ sind * 3 + 1 ] = (double) crtPoint.y;
					points[ sind * 3 + 2 ] = (double) crtPoint.z + (((double) rand() / (RAND_MAX+1))*0.01);
					sind++;
				}
			}
		}
	}

	std::cout << "sind " << sind << std::endl; 

	gdiam_point* pnt_arr;
	gdiam_bbox bb;

	pnt_arr = gdiam_convert( (gdiam_real *)points, pointcount );//sind );//num );

	printf( "Computing a tight-fitting bounding box of the point-set\n" );
	bb = gdiam_approx_mvbb_grid_sample( pnt_arr, sind, 5, 400 );//num, 5, 400 );

	std::cout<< "Done. "<<std::endl;
	
	coeffs.values.push_back(bb.getPoint(0,0,0,0));
	coeffs.values.push_back(bb.getPoint(0,0,0,1));
	coeffs.values.push_back(bb.getPoint(0,0,0,2));
	
	coeffs.values.push_back(bb.getPoint(0,0,1,0));
	coeffs.values.push_back(bb.getPoint(0,0,1,1));
	coeffs.values.push_back(bb.getPoint(0,0,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,1,0));
	coeffs.values.push_back(bb.getPoint(0,1,1,1));
	coeffs.values.push_back(bb.getPoint(0,1,1,2));

	coeffs.values.push_back(bb.getPoint(0,1,0,0));
	coeffs.values.push_back(bb.getPoint(0,1,0,1));
	coeffs.values.push_back(bb.getPoint(0,1,0,2));
	
	coeffs.values.push_back(bb.getPoint(1,0,0,0));
	coeffs.values.push_back(bb.getPoint(1,0,0,1));
	coeffs.values.push_back(bb.getPoint(1,0,0,2));

	coeffs.values.push_back(bb.getPoint(1,0,1,0));
	coeffs.values.push_back(bb.getPoint(1,0,1,1));
	coeffs.values.push_back(bb.getPoint(1,0,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,1,0));
	coeffs.values.push_back(bb.getPoint(1,1,1,1));
	coeffs.values.push_back(bb.getPoint(1,1,1,2));

	coeffs.values.push_back(bb.getPoint(1,1,0,0));
	coeffs.values.push_back(bb.getPoint(1,1,0,1));
	coeffs.values.push_back(bb.getPoint(1,1,0,2));

	//std::cout << coeffs << std::endl;
	//bb.dump();

	free(points);

	return coeffs;

}*/


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
	SegmentCloud::getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
		boost::shared_ptr<pcl::PointIndices> objectInliers,
		pcl::PointIndices::Ptr planeInliers,
		boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
	switch( crtMethod ) {
	case SegBack:
		return getMask(input, this->background, objectInliers);
		break;
	case SegObj:
		break;
	case SegPlane:
		return getPlaneMask(input, objectInliers, planeInliers, planeCoeffs, cloud_filtered);
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
	boost::shared_ptr<pcl::PointIndices> objectInliers(new pcl::PointIndices);
	pcl::PointIndices::Ptr planeInliers( new pcl::PointIndices);
	boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB> );
	return getMask(input, objectInliers, planeInliers, planeCoeffs, cloud_filtered);
}

boost::shared_ptr<cv::Mat>
	SegmentCloud::getPlaneMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,  boost::shared_ptr<pcl::PointIndices> objectinliers,
		pcl::PointIndices::Ptr planeInliers,
		boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered
		)
{

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

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
	//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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
	seg.segment (*planeInliers, *planeCoeffs);//*coefficients);
	if(planeInliers->indices.size()==0){
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

		if(!(inliercount>=planeInliers->indices.size() || planeInliers->indices.at(inliercount)==n || (imgz!=imgz))){
			//mask->data[j*mask->step[0]+i*mask->step[1]] = 255;
			mask->data[j*mask->step[0]+i] = 255;// no step 1!!
			objectinliers->indices.push_back(n);
		}
		else{

			if(inliercount<planeInliers->indices.size() && planeInliers->indices.at(inliercount)==n){inliercount++;}
		}
	}

	// Erode the mask
	//std::cout << "eroding mask"<<std::endl;
	cv::Mat el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(1,1));
	//cv::Mat el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(1,1));
	cv::erode(*mask, *mask, el);
	
	return mask;
}


boost::shared_ptr<std::vector<cv::Rect> > SegmentCloud::getROIS(boost::shared_ptr<const cv::Mat> mask)
{
	boost::shared_ptr<std::vector<cv::Rect> > rois(new std::vector<cv::Rect>);
	IplImage ipl_bmask = *mask;//cvCreateImage(cvSize(BooleanMask.size().height,BooleanMask.size().width),8,1);
	//std::cout << "Created ipl version of mask " <<  std::endl;
	cvSetImageROI(&ipl_bmask, cvRect(0,0,ipl_bmask.width, ipl_bmask.height));
	//BwImage enter(ipl_bmask);
	imcalc.Calculate(&ipl_bmask, 5);

	int minx;
	int miny;
	int mean_x;
	int mean_y;
	int boxWidth;
	int boxHeight;
	int rc;

	for(rc = 0; rc<5 ; rc++){


		mean_x = imcalc.getmean(rc,0);
		if(mean_x==0){break;}

		mean_y = imcalc.getmean(rc,1);

		boxWidth  = imcalc.getRegions(rc,0);
		boxHeight = imcalc.getRegions(rc,1);

		boxWidth = static_cast<int>(boxWidth);//*1.5);
		boxHeight = static_cast<int>(boxHeight);//*1.5);

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
	SegmentCloud::getUnorgCloud(
		const cv::Mat mask,
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
		cv::Rect ROI)
{

	int imin = ROI.x;
	int imax = imin+ROI.width;
	int jmin = ROI.y;
	int jmax = jmin+ROI.height;

	int pointcount=0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB crtPoint;
	for(int i=imin; i<imax; i+=1)//i++)
	{
		for(int j=jmin; j<jmax; j+=1)//j++)
		{
			if( mask.data[j*mask.step[0]+i] != 0 ) {
				crtPoint = input->at(i,j);

				if(crtPoint.x==crtPoint.x && crtPoint.y==crtPoint.y && crtPoint.z==crtPoint.z){
					segmentCloud->push_back(input->at(i,j));
					pointcount++;
				}
			}
		}
	}
	return segmentCloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	SegmentCloud::getUnorgCloud(boost::shared_ptr<const cv::Mat> mask,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
{
	int nmax = input->width * input->height;
	//mask.release();
	//BooleanMask = cv::Mat::ones( inputCloud->height, inputCloud->width, CV_8UC1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	/*for(int n=0; n < nmax ; n++ ){
		int i = n % input->width;
		int j = ((n-i) / input->width);

		if( mask->data[j*mask->step[0]+i] != 0 && input->at(n).x==input->at(n).x && input->at(n).y==input->at(n).y && input->at(n).z==input->at(n).z)
			segmentCloud->push_back(input->at(n));
	}*/
	pcl::PointXYZRGB crtPoint;
	int pointcount = 0;
	for( int i = 0; i < input->width; i++ ) {
		for( int j = 0; j < input->height; j++ ) {
			//if( mask->data[j*mask->step[0]+i] != 0 && input->at(i,j).x==input->at(i,j).x && input->at(i,j).y==input->at(i,j).y && input->at(i,j).z==input->at(i,j).z)
			//	segmentCloud->push_back(input->at(i,j));

			if( mask->data[j*mask->step[0]+i] != 0 ) {
				crtPoint = input->at(i,j);

				if(crtPoint.x==crtPoint.x && crtPoint.y==crtPoint.y && crtPoint.z==crtPoint.z){
					segmentCloud->push_back(input->at(i,j));
					pointcount++;
				}
			}
		}
	}
	std::cout << "unorg pt count " << pointcount << " " << segmentCloud->size() << std::endl;




	//std::vector<int> a;
	//pcl::removeNaNFromPointCloud(*segmentCloud,*segmentCloud,a);

	return segmentCloud;
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
