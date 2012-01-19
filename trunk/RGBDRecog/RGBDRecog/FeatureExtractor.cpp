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
	des = new OpponentColorDescriptorExtractor(SDE); //written by van de Sande... extracts the opponent SIFT data
	SuDE = new cv::SurfDescriptorExtractor();
	desSURF = new OpponentColorDescriptorExtractor(SuDE);

	siftdetector = new SIFT(4*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE );

	surfdetector = new SURF(4*100, 4, 2, false);;

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
			FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", FileStorage::READ);
			if(fs.isOpened()){
				cout << "- Loaded " << featureNames[i] <<  " codebook" << endl;
				Mat M; fs["codebook" + boost::lexical_cast<string>(i)] >> M;
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

void FeatureExtractor::addDescriptor(bool & firstadded, Mat & tempfeaturevector,Mat & descriptors, int mode){
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

Mat FeatureExtractor::extractFeatures(vector<bool> modes, cv::Mat rgbimg){
	if(!codebooksloaded){
		cout << "ERROR: codebooks not loaded" << endl;
		return Mat();
	}

	vector<Mat> rawfeatures = extractRawFeatures(modes,rgbimg);
	Mat featurevector;
	bool firstadded = false;
	

	//in this loop we run each Mat from the rawfeatures through the
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

Mat FeatureExtractor::normalSift(const Mat grayimg){
	Mat descriptors;
	siftdetector->operator()(grayimg,Mat(),vector<KeyPoint>(),descriptors);
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	
	return descriptors;
}
Mat FeatureExtractor::hueSift(const Mat grayimg, const Mat hueimg){
	Mat descriptors;
	vector<KeyPoint> keypoints;
	siftdetector->operator()(grayimg,Mat(),keypoints,Mat());
	siftdetector->operator()(hueimg,Mat(),keypoints,descriptors,true);
	//normalize features
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
Mat FeatureExtractor::opSift(const Mat grayimg, const Mat rgbimg){
	Mat descriptors;
	vector<KeyPoint> keypoints;
	siftdetector->operator()(grayimg,Mat(),keypoints,Mat());
	des->compute(rgbimg,keypoints,descriptors);
	for(int i = 0; i < descriptors.rows; i++){
		normalize(descriptors.row(i),descriptors.row(i));
	}
	return descriptors;
}
Mat FeatureExtractor::normalSurf(const Mat grayimg){
	vector<float> tempfloatdesc;
	vector<KeyPoint> keypoints;
	surfdetector->operator()(grayimg,Mat(),keypoints,tempfloatdesc);

	unsigned int keypointsi = keypoints.size();
	unsigned int dimensions = tempfloatdesc.size()/(keypoints.size());

	Mat descriptors = Mat(keypointsi,dimensions,CV_32FC1);

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
Mat FeatureExtractor::hueSurf(const Mat grayimg, const Mat hueimg){
	vector<float> tempfloatdesc;
	vector<KeyPoint> keypoints;
	surfdetector->operator()(grayimg,Mat(),keypoints,vector<float>());
	surfdetector->operator()(hueimg,Mat(),keypoints,tempfloatdesc,true);
	//descriptors = Mat(tempfloatdesc).reshape(1,keypoints.size());

	unsigned int keypointsi = keypoints.size();
	unsigned int dimensions = tempfloatdesc.size()/(keypoints.size());

	Mat descriptors = Mat(keypointsi,dimensions,CV_32FC1);

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
Mat FeatureExtractor::opSurf(const Mat grayimg,const Mat rgbimg){
	vector<KeyPoint> keypoints;
	Mat descriptors;
	surfdetector->operator()(grayimg,Mat(),keypoints,vector<float>());
	desSURF->compute(rgbimg,keypoints,descriptors);

	keypoints.clear();

	return descriptors;
}



vector<Mat> FeatureExtractor::extractRawFeatures(vector<bool> modes, cv::Mat rgbimg){
	vector<Mat> rawfeatures;
	Mat grayimg;
	Mat hueimg;
	//for these modes we use hue information, so we also make the hueimg
	//We also get the grayimg for free :)
	if(modes[1] || modes[4]){
		Mat tempimg;
		vector<Mat> planes;
		cvtColor(rgbimg,tempimg,CV_BGR2HSV);
		split(rgbimg,planes);
		hueimg = planes[0];
		grayimg = planes[2];

		tempimg.release();
		planes.clear();
	}else{ //else we only need the gray img ^_^
		cvtColor(rgbimg,grayimg,CV_BGR2GRAY);
	}
	

	//for each mode, add the features for this picture to a matrix
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