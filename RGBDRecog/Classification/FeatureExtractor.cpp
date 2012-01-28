
#include "StdAfx.h"
#include "FeatureExtractor.h"



#define SATURATION_THRESH 0.1
#define VALUE_THRESH 0.1

FeatureExtractor::FeatureExtractor(void)
{
	cout << "Initializing FeatureExtractor" << endl;

	codebooksloaded = false;

	featureData = new FeatureData();

	//init descriptors
	SDE = new cv::SiftDescriptorExtractor(); //Sift descriptor extraction class
	des = new cv::OpponentColorDescriptorExtractor(SDE); //implementation by van de Sande... extracts the opponent SIFT data
	SuDE = new cv::SurfDescriptorExtractor();
	desSURF = new cv::OpponentColorDescriptorExtractor(SuDE);

	//sift detector with specified standard parameters
	// the 4* in the first parameter indicates the treshold, higher means less decriptors found
	siftdetector = new cv::SIFT((0.04 / cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
          cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          cv::SIFT::CommonParams::FIRST_ANGLE );
	
	//surf detector with standard parameters, 
	// the 4* in the first parameter indicates the treshold, higher means less decriptors found
	surfdetector = new cv::SURF(100, 4, 2, false);;

	cout << "Done initializing FeatureExtractor" << endl;
}

FeatureExtractor::~FeatureExtractor(void)
{
	delete featureData;
}

// this function loads the codebooks in the format generated by the RGBDRecog project
// see the function for the naming details
void FeatureExtractor::loadCodebooks(){
	cout << "Loading the codebooks" << endl;
	codebooks = new FeatureVector[featureData->amountOfFeatures]; 
		for(int i = 0; i < featureData->amountOfFeatures; i++){
			cv::FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
			if(fs.isOpened()){
				cout << "- Loaded " << featureData->featureNames[i] <<  " codebook" << endl;
				cv::Mat M; fs["codebook" + boost::lexical_cast<string>(i)] >> M;
				fs.release();
				codebooks[i].AddFeatures(M);
				codebooks[i].TrainkNN();
			}else{
				cout << "- " << featureData->featureNames[i]  << " codebook does not exist" << endl;
			}
	}
	cout << "Loaded the available codebooks" << endl;
	codebooksloaded = true;
}

void FeatureExtractor::loadCodebooks(string filepath){
	cout << "Loading the codebooks from location " << filepath << endl;
	codebooks = new FeatureVector[featureData->amountOfFeatures]; 
		for(int i = 0; i < featureData->amountOfFeatures; i++){
			cv::FileStorage fs(filepath + "codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
			if(fs.isOpened()){
				cout << "- Loaded " << featureData->featureNames[i] <<  " codebook" << endl;
				cv::Mat M; fs[filepath + "codebook" + boost::lexical_cast<string>(i)] >> M;
				fs.release();
				codebooks[i].AddFeatures(M);
				codebooks[i].TrainkNN();
			}else{
				cout << "- " << featureData->featureNames[i]  << " codebook does not exist" << endl;
			}
	}
	cout << "Loaded the available codebooks" << endl;
	codebooksloaded = true;
}

// add descriptor descriptors to tempfeaturevector
// function also runs the descriptors throught he codebooks, making a histogram out of them
// mode is the type of feature, firstadded simply indicates if this was the first added feature
void FeatureExtractor::addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector,cv::Mat & descriptors, int mode){
	if(mode >= 0 && mode <=5){//if this is a codebook feature
		if(codebooks[mode].features->empty()){
			cout << "ERROR: Codebook" << featureData->featureNames[mode] << " does not exist";
			return;
		}
	}

	if(!firstadded){
		tempfeaturevector.release();
		if(mode >= 0 && mode <=5){
			tempfeaturevector = codebooks[mode].GenHistogram(descriptors);
		}else if (mode ==  6){ //color histogram
			tempfeaturevector = descriptors;
		}
		firstadded = true;
	}else{
		if(mode >= 0 && mode <=5){
			vconcat(tempfeaturevector,codebooks[mode].GenHistogram(descriptors),tempfeaturevector);
		}else if (mode == 6) {
			vconcat(tempfeaturevector,descriptors,tempfeaturevector);
		}
	}
}

cv::Mat FeatureExtractor::normalSift(const cv::Mat grayimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	cv::Mat descriptors;

	#ifdef DENSE_SAMPLING
	siftdetector->operator()(grayimg,mask,keypoints,descriptors,true); //use the given keypoints
	#else
	siftdetector->operator()(grayimg,mask,keypoints,descriptors);
	#endif
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	
	return descriptors;
}
cv::Mat FeatureExtractor::hueSift(const cv::Mat grayimg, const cv::Mat hueimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	cv::Mat descriptors;
#ifdef DENSE_SAMPLING
	siftdetector->operator()(hueimg,mask,keypoints,descriptors,true);
#else
	siftdetector->operator()(grayimg,mask,keypoints,cv::Mat());
	siftdetector->operator()(hueimg,mask,keypoints,descriptors,true);
#endif
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
cv::Mat FeatureExtractor::opSift(const cv::Mat grayimg, const cv::Mat rgbimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	cv::Mat descriptors;
#ifdef DENSE_SAMPLING
	//do nothing
#else
	siftdetector->operator()(grayimg,mask,keypoints,cv::Mat());
#endif
	
	des->compute(rgbimg,keypoints,descriptors);
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
cv::Mat FeatureExtractor::normalSurf(const cv::Mat grayimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	vector<float> tempfloatdesc;
#ifdef DENSE_SAMPLING
	surfdetector->operator()(grayimg,mask,keypoints,tempfloatdesc,true);
#else
	surfdetector->operator()(grayimg,mask,keypoints,tempfloatdesc);
#endif
	

	unsigned int keypointsi = keypoints.size();
	unsigned int dimensions = tempfloatdesc.size()/(keypoints.size());

	cv::Mat descriptors = cv::Mat(keypointsi,dimensions,CV_32FC1);

	//hard copy the vector... opencv reshape functions suck apparantly
	for(unsigned int i = 0; i < keypointsi; i++){
		for(unsigned int j = 0; j < dimensions; j++){
			descriptors.at<float>(i,j) = tempfloatdesc[i*dimensions+j];
		}
	}
	tempfloatdesc.clear();
	keypoints.clear();
	return descriptors;
}
cv::Mat FeatureExtractor::hueSurf(const cv::Mat grayimg, const cv::Mat hueimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	vector<float> tempfloatdesc;
#ifdef DENSE_SAMPLING
	surfdetector->operator()(hueimg,mask,keypoints,tempfloatdesc,true);
#else
	surfdetector->operator()(grayimg,mask,keypoints,vector<float>());
	surfdetector->operator()(hueimg,mask,keypoints,tempfloatdesc,true);
#endif

	//descriptors = cv::Mat(tempfloatdesc).reshape(1,keypoints.size());

	unsigned int keypointsi = keypoints.size();
	unsigned int dimensions = tempfloatdesc.size()/(keypoints.size());

	cv::Mat descriptors = cv::Mat(keypointsi,dimensions,CV_32FC1);

	//hard copy the vector... opencv reshape functions suck apparantly
	for(unsigned int i = 0; i < keypointsi; i++){
		for(unsigned int j = 0; j < dimensions; j++){
			descriptors.at<float>(i,j) = tempfloatdesc[i*dimensions+j];
		}
	}

	keypoints.clear();
	tempfloatdesc.clear();

	return descriptors;
}
cv::Mat FeatureExtractor::opSurf(const cv::Mat grayimg,const cv::Mat rgbimg, const cv::Mat mask = cv::Mat(),vector<cv::KeyPoint> keypoints){
	cv::Mat descriptors;
#ifdef DENSE_SAMPLING
	surfdetector->operator()(grayimg,mask,keypoints,vector<float>(),true);
#else
	surfdetector->operator()(grayimg,mask,keypoints,vector<float>());
#endif
	
	desSURF->compute(rgbimg,keypoints,descriptors);

	keypoints.clear();

	return descriptors;
}

cv::Mat FeatureExtractor::colorHistogramCreator(vector<cv::Mat> hsvPlanes){

	cv::Mat maskSat = hsvPlanes[1] > SATURATION_THRESH*256;

	cv::Mat maskVal = hsvPlanes[2] > VALUE_THRESH*256;
	cv::Mat mask = cv::Mat(hsvPlanes[0].size(),CV_8UC1);

	

	cv::bitwise_and(maskSat, maskVal, mask); //create hue/saturation mask
    // let's quantize the hue to 30 levels
    int hbins = 30;
    int histSize[] = {hbins};
    // hue varies from 0 to 180, using integer images. see cvtColor
    float hranges[] = { 0, 180 };

    const float* ranges[] = {hranges};
    cv::Mat hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};
    cv::calcHist( &hsvPlanes[0], 1, channels, mask, // use mask
        hist, 1, histSize, ranges,
        true, // the histogram is uniform
        false );

	cv::normalize(hist,hist);
	maskSat.release();
	maskVal.release();
	mask.release();

	cv::Mat meanvar(2,1,CV_32FC1);
	float mean = 0; //calculate mean
	for(int i = 0; i < hbins; i++){
		mean += i*hist.at<float>(i);
	}
	meanvar.at<float>(0) = mean;
	//calculate variance
	float var = 0;
	for(int i = 0; i < hbins; i++){
		var+= i* (hist.at<float>(i) - mean)*(hist.at<float>(i) - mean);
	}


	return hist;
}

vector<cv::Mat> FeatureExtractor::extractRawFeatures(vector<bool> modes, cv::Mat rgbimg, const cv::Mat mask){
	vector<cv::Mat> rawfeatures;
	cv::Mat grayimg;
	cv::Mat hueimg;
	vector<cv::Mat> planes;
	//for these modes we use hue inforcv::Mation, so we also make the hueimg
	//We also get the grayimg for free :)
	if(modes[1] || modes[4] || modes[6]){
		cv::Mat tempimg;
		cvtColor(rgbimg,tempimg,CV_BGR2HSV);
		split(tempimg,planes);
		hueimg = planes[0];
		grayimg = planes[2];
		tempimg.release();
	}else{ //else we only need the gray img ^_^
		cvtColor(rgbimg,grayimg,CV_BGR2GRAY);
	}
	
	#ifdef DENSE_SAMPLING
	vector<cv::KeyPoint> keypoints;
	cv::DenseFeatureDetector fd;
	fd.detect(rgbimg,keypoints);
	#endif

	//for each mode, add the features for this picture to a cv::Matrix
	rawfeatures.clear();
	if(modes[0]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(normalSift(grayimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(normalSift(grayimg));
	#endif
		
	}
	if(modes[1]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(hueSift(grayimg,hueimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(hueSift(grayimg,hueimg));
	#endif
		
	}
	if(modes[2]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(opSift(grayimg,rgbimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(opSift(grayimg,rgbimg));
	#endif
		
	}
	if(modes[3]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(normalSurf(grayimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(normalSurf(grayimg));
	#endif
		
	}
	if(modes[4]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(hueSurf(grayimg,hueimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(hueSurf(grayimg,hueimg));
	#endif
		
	}
	if(modes[5]){
	#ifdef DENSE_SAMPLING
		rawfeatures.push_back(opSurf(grayimg,rgbimg,cv::Mat(),keypoints));
	#else
		rawfeatures.push_back(opSurf(grayimg,rgbimg));
	#endif	
	}
	if(modes[6]){ //not mask ready unfortunately
		rawfeatures.push_back(colorHistogramCreator(planes));
	}

	grayimg.release();
	hueimg.release();
	planes.clear();

	return rawfeatures;
}
vector<cv::Mat> FeatureExtractor::extractRawFeatures(vector<bool> modes, cv::Mat rgbimg, cv::Rect roi){
	cv::Mat roiimg = rgbimg(roi);
	return extractRawFeatures(modes, roiimg);
}

cv::Mat FeatureExtractor::extractFeatures(vector<bool> modes, cv::Mat rgbimg, cv::Mat mask){
	if(!codebooksloaded){
		cout << "ERROR: codebooks not loaded" << endl;
		return cv::Mat();
	}

	vector<cv::Mat> rawfeatures = extractRawFeatures(modes,rgbimg);
	cv::Mat featurevector;
	bool firstadded = false;
	

	//in this loop we run each cv::Mat from the rawfeatures through the
	//respective codebooks
	int addedFeatures = 0;
	for(int i = 0; i < featureData->amountOfFeatures; i++){
		if(modes[i] && rawfeatures[addedFeatures].rows>0){
			addDescriptor(firstadded, featurevector, rawfeatures[addedFeatures], i);
			addedFeatures++;
		}
	}

	return featurevector;
}


cv::Mat FeatureExtractor::extractFeatures(vector<bool> modes, cv::Mat rgbimg, cv::Rect roi){
	if(roi.height < 0 || roi.height > 0){
		cout << "Error: invalid ROI" << endl;
		return cv::Mat();
	}
	cv::Mat roiimg = rgbimg(roi);
	return extractFeatures(modes, roiimg);
}


cv::Mat FeatureExtractor::createMask(int rows, int cols, cv::Rect box){
	cv::Mat mask = cv::Mat::ones(rows,cols,CV_8UC1);
	for(int i = box.x; i < box.width; i++){
		for(int j = box.y; j < box.height; j++){
			mask.at<int>(i,j) = 0;
		}
	}
	return mask;
}

boost::shared_ptr<cv::Mat> FeatureExtractor::cloudToRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	boost::shared_ptr<cv::Mat> rgbImage(new cv::Mat(cloud->height, cloud->width, CV_8UC3));
	for (unsigned int x = 0; x < cloud->width; x++)
	{
		for (unsigned int y = 0; y < cloud->height; y++)
		{
			rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 2] = (*cloud)(x, y).r;
			rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 1] = (*cloud)(x, y).g;
			rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 0] = (*cloud)(x, y).b;
		}
	}
	return rgbImage;
}


//pcl::PointCloud<pcl::Normal>::Ptr FeatureExtractor::calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
//	 Create the normal estimation class, and pass the input dataset to it
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setInputCloud (cloud);
//
//	 Create an empty kdtree representation, and pass it to the normal estimation object.
//	 Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//	ne.setSearchMethod (tree);
//
//	 Output datasets
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//
//	 Use all neighbors in a sphere of radius 3cm
//	ne.setRadiusSearch (0.03);
//
//	 Compute the features
//	ne.compute (*cloud_normals);
//
//	return cloud_normals;
//}
//
//boost::shared_ptr<cv::Mat> FeatureExtractor::calculateFPFH(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals){
//	 Create the FPFH estimation class, and pass the input dataset+normals to it
//	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
//	fpfh.setInputCloud (cloud);
//	fpfh.setInputNormals (normals);
//	 alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
//
//	 Create an empty kdtree representation, and pass it to the FPFH estimation object.
//	 Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//	fpfh.setSearchMethod (tree);
//
//	 Output datasets
//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
//
//	 Use all neighbors in a sphere of radius 5cm
//	 IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//	fpfh.setRadiusSearch (0.05);
//
//	 Compute the features
//	fpfh.compute (*fpfhs);
//		
//	 Convert to cv::Mat (there's probably a faster way: memcpy innerloop; )
//	boost::shared_ptr<cv::Mat> outMat( new cv::Mat(33, fpfhs->points.size(), CV_32FC1) );
//	for (unsigned int p = 0; p < fpfhs->points.size(); p++)
//	{
//		for (unsigned int f = 0; f < 33; f++){
//			outMat->data[outMat->step[0]*f + p] = fpfhs->at(p).histogram[f];
//		}
//	}
//	return outMat;
//}
