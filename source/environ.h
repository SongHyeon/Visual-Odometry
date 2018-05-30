#ifndef ENVIRON_H
#define ENVIRON_H


#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>  
#include <list>
#include "use_opencv.h"
#include "GPSParser.h"
#include "visual_odometry.h"

#define ROI_START_X 0
#define ROI_START_Y 200

#define ROI_COLS 800
#define ROI_ROWS 340

#define FOCAL_LENGTH_X 877.3481
#define FOCAL_LENGTH_Y 879.3675
#define CU 420.5168
#define CV 329.6696

const double A2R = M_PI / 180.0;
const double R2A = 180.0 / M_PI;

enum class LOG_DATA_TYPE{LIDAR,CAMERA,GPS,VEH_SPEED};

typedef struct LogData{
	LOG_DATA_TYPE dataType;
	int timeStamp;
	int frameNum;
	int speed;
	vector<vector<int>> lidarData;
	GpsData gpsData;
	LogData() :  timeStamp(0), frameNum(0), speed(0)
	{}
}LogData;
#endif 