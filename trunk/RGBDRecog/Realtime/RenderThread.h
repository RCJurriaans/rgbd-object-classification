
#pragma once

#include "stdafx.h"
#include "ClassificationResults.h"



class RenderThread
{
public:
	
	RenderThread( boost::shared_ptr<ClassificationResults> res ) :
	  viewer(NULL)
	  ,results(res)
	{
		// Not a RAII constructor, but I think this would mess up thread safety
		cout << "RenderThread constructed" << endl;

		ifstream infile;
		std::string filename;
		filename += getenv("RGBDDATA_DIR") ;
		filename += "\\data_info.txt";
 		infile.open(filename); //open data_info.txt etc.
		if(!infile.good()){
			cout << "Could not find %RGBDATA_DIR%/data_info.txt" << endl;
			return;
		}
		std::string temps; //temporary read variables

		char temp[256];
		int tempi;
		infile.getline(temp,256);

		int amountOfClasses;
		infile >> amountOfClasses; //amount of classes in dataset
		std::string fileExtension;
		infile >> fileExtension;

		//get the names of the classes
		for(int i = 0; i < amountOfClasses; i++){
			infile >> temps;
			classNames.push_back(temps);
		}
		/*
		classNames.push_back("BrownBanana");
		classNames.push_back("Coconut");
		classNames.push_back("DoraPasta");
		classNames.push_back("Lemon");
		classNames.push_back("MoldyRice");
		classNames.push_back("Mustard");
		classNames.push_back("Pepper");
		classNames.push_back("RottenCheese");
		classNames.push_back("RottenChicory");
		classNames.push_back("SourMilk");
		classNames.push_back("Spaghetti");
		classNames.push_back("Taco");*/
	}

	void visCallback(pcl::visualization::PCLVisualizer& vis);
	void run();

protected:
	pcl::visualization::CloudViewer* viewer;

	boost::shared_ptr<ClassificationResults> results;

	
	std::vector<std::string> classNames;

private:
	RenderThread( const RenderThread& other ); // non construction-copyable
    RenderThread& operator=( const RenderThread& ); // non copyable
};
