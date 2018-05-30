
#include "visual_odometry.h"

void VO::init(double focal, double cx, double cy){
	this->focal = focal;
	this->pp.x = cx;
	this->pp.y = cy;
}

void VO::featureTracking(Mat &img_1, Mat &img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

	//this function automatically gets rid of points for which tracking fails

	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++)
	{
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0) || (pt.x >= img_1.cols) || (pt.y >= img_1.rows))	{
			//if ((pt.x<0) || (pt.y<0))	{
			status.at(i) = 0;
			//}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}

	}

}


void VO::featureDetection(Mat &img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

//Gray
bool VO::calcMotion(Mat &prevImage, Mat &currImage, Mat &R, Mat &t,double scale){
	Mat E;
	Mat R_f, t_f;
	vector<uchar> status;
	if (prevFeatures.size() < MIN_NUM_FEAT)	{
		//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
		//cout << "trigerring redection" << endl;
		featureDetection(prevImage, prevFeatures);
		//featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
	}
	//featureDetection(prevImage, prevFeatures);
	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
	double sumDist = 0;
	for (int i = 0; i < prevFeatures.size(); ++i){
		double dx, dy;
		dx = prevFeatures[i].x - currFeatures[i].x;
		dy = prevFeatures[i].y - currFeatures[i].y;
		sumDist += dx*dx + dy*dy;
	}

	if (scale < 1 && sumDist / prevFeatures.size() < 400){
		return false;
	}

	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0);
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp);

	Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);


	for (int i = 0; i < prevFeatures.size(); i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
		prevPts.at<double>(0, i) = prevFeatures.at(i).x;
		prevPts.at<double>(1, i) = prevFeatures.at(i).y;

		currPts.at<double>(0, i) = currFeatures.at(i).x;
		currPts.at<double>(1, i) = currFeatures.at(i).y;
	}

	//scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

	//cout << "Scale is " << scale << endl;

	//if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
	//	t_f = t_f + scale*(R_f*t);
	//	R_f = R*R_f;
	//}else {
	//	//cout << "scale below 0.1, or incorrect translation" << endl;
	//}

	// lines for printing results
	// myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

	// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
	

/*
	for (int i = 0; i < prevFeatures.size(); i++){
		line(currImage_c, prevFeatures[i], currFeatures[i], Scalar(0, 0, 255));
		circle(currImage_c, currFeatures[i], 2, Scalar(0, 0, 255), -1);
	}
*/

	//prevImage = currImage.clone();
	/*prevFeatures = currFeatures;*/

	//int x = int(t_f.at<double>(0)) + 300;
	//int y = int(t_f.at<double>(2)) + 100;
	//circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

	//rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
	//sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
	//putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

	//imshow("Road facing camera", currImage_c);
	//imshow("Trajectory", traj);

	//waitKey(1);

	return true;
}

void VO::updateFeatures(){

	prevFeatures = currFeatures;
}