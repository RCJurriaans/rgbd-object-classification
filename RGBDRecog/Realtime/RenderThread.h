
#pragma once

#include "stdafx.h"
#include "ClassificationResults.h"




class RenderThread
{
public:
	
	RenderThread( boost::shared_ptr<ClassificationResults> res ) :
	  viewer(NULL)
	  ,results(res),
	  rightPressed(false), leftPressed(false), upPressed(false), downPressed(false), shiftPressed(false)
	{
		// Not a RAII constructor, but I think this would mess up thread safety
		cout << "RenderThread constructed" << endl;

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
		classNames.push_back("Taco");
	}

	void visCallback(pcl::visualization::PCLVisualizer& vis);
	void run();

	bool rightPressed;
	bool leftPressed;
	bool upPressed;
	bool downPressed;
	bool shiftPressed;

protected:
	pcl::visualization::CloudViewer* viewer;

	boost::shared_ptr<ClassificationResults> results;

	
	std::vector<std::string> classNames;

private:
	RenderThread( const RenderThread& other ); // non construction-copyable
    RenderThread& operator=( const RenderThread& ); // non copyable
};
