#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H
#include "use_opencv.h"
#include "environ.h"

#define MIN_NUM_FEAT 2000

class VO{

public :
	VO(){};
	~VO(){};
	
	double focal;
	Point2d pp;
	vector<Point2f> prevFeatures;
	vector<Point2f> currFeatures;

	void init(double focal, double cx, double cy);
	void featureTracking(Mat &img_1, Mat &img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);
	void featureDetection(Mat &img_1, vector<Point2f>& points1);
	bool calcMotion(Mat &prevImage, Mat &currImage, Mat &R, Mat &t, double scale);
	void updateFeatures();
};
#endif