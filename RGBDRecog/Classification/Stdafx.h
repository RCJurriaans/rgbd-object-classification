// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once


#include <stdio.h>
#include <tchar.h>
//#define _AFXDLL
//#include <afx.h>

#include <boost\lexical_cast.hpp>
//#include "Winbase.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/video.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/random_sample.h>