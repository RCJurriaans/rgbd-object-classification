# Introduction #

How to get RGBDRecog compilable and working

WARNING: Reset visual studio every time you set environment variables.


# Details #

## OpenCV Install ##
  * Download and install openCV 2.3.1 from http://opencv.willowgarage.com/wiki/InstallGuide
  * Follow the cmake/compile instructions as listed in the above link. Make sure that you use the extracted folder as the source and target paths for cmake. Do not change anything in the cmake settings and make sure you compile for the win32 debug and release versions.
  * It is important that you add cmake\_binary\_dir\bin\Release,cmake\_binary\_dir\bin\Debug to the path directory
  * Set a new system variable OPENCV\_DIR to the install directory of opencv.
  * Run the config.bat file in the main RGBDRecog folder

## Boost Install ##
  * Download and extract boost 1.48.0 to any location (http://www.boost.org/)
  * Set system or user variable BOOST\_ROOT to the folder's root (e.g. C:\boost\_1\_48\_0)

## Data path ##
  * Set system or user variable RGBDDATA\_DIR to the folder containing the training data.