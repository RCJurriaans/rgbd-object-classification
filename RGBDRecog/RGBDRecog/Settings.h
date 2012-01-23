#pragma once

#include <FeatureData.h>

using namespace std;
class Settings
{
public:
	Settings(void);
	~Settings(void);

	//display the menu to change the settings
	void menu();

	//output the current settings to the screen
	void outputSettings();

	//return a string that contains all settings information compactly
	string settingsString();

	//actual settings info
	//readily accessible
	bool segmentation;  //true if segmentation is used, false otherwise
	vector<bool> modes; //see Classification : FeatureData for the features used
						//true if the feature is used, false otherwise

private:
	bool readSettings();
	bool writeSettings();
	string boolstring(bool in);
	void segmentationSettings();
	void Settings::featureSettings();
	string settingsfile;



	FeatureData * featureData;

};

