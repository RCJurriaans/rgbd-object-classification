#include "StdAfx.h"
#include "FeatureExtractor.h"


FeatureExtractor::FeatureExtractor(void)
{
	cout << "Initializing FeatureExtractor" << endl;

	codebooksloaded = false;
	amountOfFeatures = 6;
	featureNames = new string[6];
	featureNames[0] = "NORMALSIFT";
	featureNames[1] = "HUESIFT";
	featureNames[2] = "OPPONENTSIFT";
	featureNames[3] = "NORMALSURF";
	featureNames[4] = "HUESURF";
	featureNames[5] = "OPPONENTSURF";
	SDE = new cv::SiftDescriptorExtractor(); //Sift descriptor extraction class
	des = new cv::OpponentColorDescriptorExtractor(SDE); //written by van de Sande... extracts the opponent SIFT data
	SuDE = new cv::SurfDescriptorExtractor();
	desSURF = new cv::OpponentColorDescriptorExtractor(SuDE);

	siftdetector = new cv::SIFT(4*(0.04 / cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
          cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          cv::SIFT::CommonParams::FIRST_ANGLE );

	surfdetector = new cv::SURF(4*100, 4, 2, false);;

	cout << "Done initializing FeatureExtractor" << endl;
}

FeatureExtractor::~FeatureExtractor(void)
{
	delete [] featureNames;
}

void FeatureExtractor::loadCodebooks(){
	cout << "Loading the codebooks" << endl;
	codebooks = new FeatureVector[amountOfFeatures]; 
		for(int i = 0; i < amountOfFeatures; i++){
			cv::FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
			if(fs.isOpened()){
				cout << "- Loaded " << featureNames[i] <<  " codebook" << endl;
				cv::Mat M; fs["codebook" + boost::lexical_cast<string>(i)] >> M;
				fs.release();
				codebooks[i].AddFeatures(M);
				codebooks[i].TrainkNN();
			}else{
				cout << "- " << featureNames[i]  << " codebook does not exist" << endl;
			}
	}
	cout << "Loaded the available codebooks" << endl;
	codebooksloaded = true;
}

void FeatureExtractor::addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector,cv::Mat & descriptors, int mode){
	if(codebooks[mode].features->empty()){
		cout << "ERROR: Codebook" << featureNames[mode] << " does not exist";
		return;
	}

	if(!firstadded){
		tempfeaturevector.release();
		tempfeaturevector = codebooks[mode].GenHistogram(descriptors);
		firstadded = true;
	}else{
		vconcat(tempfeaturevector,codebooks[mode].GenHistogram(descriptors),tempfeaturevector);
	}
}

cv::Mat FeatureExtractor::extractFeatures(vector<bool> modes, cv::Mat rgbimg){
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
	for(int i = 0; i < amountOfFeatures; i++){
		if(modes[i]){
			addDescriptor(firstadded, featurevector, rawfeatures[addedFeatures], i);
			addedFeatures++;
		}
	}

	return featurevector;
}

cv::Mat FeatureExtractor::normalSift(const cv::Mat grayimg){
	cv::Mat descriptors;
	siftdetector->operator()(grayimg,cv::Mat(),vector<cv::KeyPoint>(),descriptors);
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	
	return descriptors;
}
cv::Mat FeatureExtractor::hueSift(const cv::Mat grayimg, const cv::Mat hueimg){
	cv::Mat descriptors;
	vector<cv::KeyPoint> keypoints;
	siftdetector->operator()(grayimg,cv::Mat(),keypoints,cv::Mat());
	siftdetector->operator()(hueimg,cv::Mat(),keypoints,descriptors,true);
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
cv::Mat FeatureExtractor::opSift(const cv::Mat grayimg, const cv::Mat rgbimg){
	cv::Mat descriptors;
	vector<cv::KeyPoint> keypoints;
	siftdetector->operator()(grayimg,cv::Mat(),keypoints,cv::Mat());
	des->compute(rgbimg,keypoints,descriptors);
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
cv::Mat FeatureExtractor::normalSurf(const cv::Mat grayimg){
	vector<float> tempfloatdesc;
	vector<cv::KeyPoint> keypoints;
	surfdetector->operator()(grayimg,cv::Mat(),keypoints,tempfloatdesc);

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
cv::Mat FeatureExtractor::hueSurf(const cv::Mat grayimg, const cv::Mat hueimg){
	vector<float> tempfloatdesc;
	vector<cv::KeyPoint> keypoints;
	surfdetector->operator()(grayimg,cv::Mat(),keypoints,vector<float>());
	surfdetector->operator()(hueimg,cv::Mat(),keypoints,tempfloatdesc,true);
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
cv::Mat FeatureExtractor::opSurf(const cv::Mat grayimg,const cv::Mat rgbimg){
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	surfdetector->operator()(grayimg,cv::Mat(),keypoints,vector<float>());
	desSURF->compute(rgbimg,keypoints,descriptors);

	keypoints.clear();

	return descriptors;
}



vector<cv::Mat> FeatureExtractor::extractRawFeatures(vector<bool> modes, cv::Mat rgbimg){
	vector<cv::Mat> rawfeatures;
	cv::Mat grayimg;
	cv::Mat hueimg;
	//for these modes we use hue inforcv::Mation, so we also make the hueimg
	//We also get the grayimg for free :)
	if(modes[1] || modes[4]){
		cv::Mat tempimg;
		vector<cv::Mat> planes;
		cvtColor(rgbimg,tempimg,CV_BGR2HSV);
		split(rgbimg,planes);
		hueimg = planes[0];
		grayimg = planes[2];

		tempimg.release();
		planes.clear();
	}else{ //else we only need the gray img ^_^
		cvtColor(rgbimg,grayimg,CV_BGR2GRAY);
	}
	

	//for each mode, add the features for this picture to a cv::Matrix
	rawfeatures.clear();
	if(modes[0]){
		rawfeatures.push_back(normalSift(grayimg));
	}
	if(modes[1]){
		rawfeatures.push_back(hueSift(grayimg,hueimg));
	}
	if(modes[2]){
		rawfeatures.push_back(opSift(grayimg,rgbimg));
	}
	if(modes[3]){
		rawfeatures.push_back(normalSurf(grayimg));
	}
	if(modes[4]){
		rawfeatures.push_back(hueSurf(grayimg,hueimg));
	}
	if(modes[5]){
		rawfeatures.push_back(opSurf(grayimg,rgbimg));
	}

	grayimg.release();
	hueimg.release();

	return rawfeatures;
}