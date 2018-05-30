#pragma once
#include "environ.h"

#define MAX_FRAME 50000
#define MIN_NUM_FEAT 2000
#define SKIP_FRAME 0
#define INTERVAL_FRAME 3


#define INF 100000

#define DIR "K:\\Users\\songhyeon\\Desktop\\program\\LidarCameraCali\\data_2cycle\\"
//#define DIR "K:\\Users\\songhyeon\\Desktop\\program\\LidarCameraCali\\data_slow\\"
//#define DIR "K:\\Users\\songhyeon\\Desktop\\program\\LidarCameraCali\\data_fast\\"
#define IMG_TYPE ".png"
//
//#define DIR "K:\\Users\\songhyeon\\Desktop\\program\\LidarCameraCali\\data_4\\"
//#define DIR "K:\\Users\\songhyeon\\Desktop\\program\\LidarCameraCali\\data_5\\"
//#define IMG_TYPE ".bmp"

void loadGpsLog(vector<LogData> &log, string Dir){
	CGPSParser gpsParser;
	FILE *File;
	File = fopen(Dir.c_str(), "r");
	if (File == NULL){
		printf("File not Opened");
		return;
	}
	int timeStamp;
	char buffer[1000];

	while (fscanf(File, "%d %s", &timeStamp, buffer) != EOF){
		GpsData gpsData;
		LogData data;
		gpsParser.parser(gpsData, string(buffer));
		data.timeStamp = timeStamp;
		data.gpsData = gpsData;
		data.dataType = LOG_DATA_TYPE::GPS;
		log.push_back(data);
	}
	fclose(File);
}

void loadCamLog(vector<LogData> &log, string Dir){
	int frameNum = 0;
	FILE *File;
	File = fopen(Dir.c_str(), "r");
	if (File == NULL){
		printf("File not Opened");
		return;
	}
	int startTime, endTime;
	while (fscanf(File, "%d %d", &startTime, &endTime) != EOF){
		LogData data;
		data.timeStamp = startTime;
		data.frameNum = frameNum++;
		data.dataType = LOG_DATA_TYPE::CAMERA;
		log.push_back(data);
	}
	fclose(File);
}

void loadLidarLog(vector<LogData> &log, string Dir, int lidarNum){
	int frameNum = 0;

	vector<FILE *> filePtr(lidarNum, NULL);
	for (int Li = 0; Li < lidarNum; Li++){
		string lidarDir = format("%s%d.txt", Dir.c_str(), Li);
		filePtr[Li] = fopen(lidarDir.c_str(), "r");
		if (filePtr[Li] == NULL){
			printf("File not Opened");
			return;
		}
	}

	int timeStamp, dataCnt;
	while (true){
		bool isEnd = false;
		for (int Li = 0; Li < lidarNum; Li++){
			isEnd = isEnd || feof(filePtr[Li]);
			//printf("%d] %d %d\n",Li, isEnd, feof(filePtr[Li]));
		}
		if (isEnd) break;
		LogData data;
		vector<vector<int>> allLidarData;
		for (int Li = 0; Li < lidarNum; Li++){
			fscanf(filePtr[Li], "%d %*lf %*lf %*lf %*d %d", &timeStamp, &dataCnt);
			vector<int> lidarData(dataCnt);
			for (int i = 0, val; i < dataCnt; i++){
				fscanf(filePtr[Li], " %d", &lidarData[i]);
			}
			allLidarData.push_back(lidarData);

		}
		data.timeStamp = timeStamp;
		data.lidarData = allLidarData;
		data.dataType = LOG_DATA_TYPE::LIDAR;
		log.push_back(data);
	}

	for (int Li = 0; Li < lidarNum; Li++){
		fclose(filePtr[Li]);
	}
}

void loadVehSpeedLog(vector<LogData> &log, string Dir){
	FILE *File;
	File = fopen(Dir.c_str(), "r");
	if (File == NULL){
		printf("File not Opened");
		return;
	}
	int timeStamp, speed;
	while (fscanf(File, "%d %d", &timeStamp, &speed) != EOF){
		LogData data;
		data.timeStamp = timeStamp;
		data.speed = speed;
		data.dataType = LOG_DATA_TYPE::VEH_SPEED;
		log.push_back(data);
	}
	fclose(File);
}

void showLidarMap(vector<vector<int>> &lidarData, Mat &Lidar){
	std::vector<cv::Scalar> vcolor;
	vcolor = std::vector<cv::Scalar>(5);
	vcolor[0] = cv::Scalar(0, 0, 255);
	vcolor[1] = cv::Scalar(255, 0, 0);
	vcolor[2] = cv::Scalar(0, 255, 0);
	vcolor[3] = cv::Scalar(255, 255, 0);
	vcolor[4] = cv::Scalar(255, 0, 255);

	double pixelPerMeter = 10;
	int LIDARNUM = lidarData.size();
	if (LIDARNUM <= 0){
		return;
	}
	int DATANUM = lidarData[0].size();
	const double dRadian = 180.0 / (DATANUM - 1) * A2R;

	Lidar.setTo(255);
	for (int i = 1; i < 60; i++){
		int row = Lidar.rows - 1 - (i * pixelPerMeter);
		int col = Lidar.cols;
		cv::line(Lidar, cv::Point(0, row), cv::Point(col, row), cv::Scalar(150, 150, 150));
		if (i % 10 == 0)
			cv::line(Lidar, cv::Point(0, row), cv::Point(col, row), cv::Scalar(0, 0, 0));
	}
	double radi;
	double lidarZ;
	double lidarX;
	int row, col;
	for (int Li = 0; Li < LIDARNUM; ++Li){
		for (int i = 0; i < DATANUM; i++){
			if (lidarData[Li][i] == 0){
				continue;
			}
			radi = dRadian * i;
			lidarZ = sin(radi)*(lidarData[Li][i] / 100.);
			lidarX = cos(radi)*(lidarData[Li][i] / 100.);

			row = Lidar.rows - 1 - (lidarZ * pixelPerMeter);
			col = lidarX * pixelPerMeter + 100;

			cv::circle(Lidar, { col, row }, .1, vcolor[Li], -1);
		}
	}
	cv::line(Lidar, cv::Point(Lidar.cols / 2, 0), cv::Point(Lidar.cols / 2, Lidar.rows), cv::Scalar(0, 0, 0));

	//cv::imshow(str_Lidar, Lidar);
	//cv::waitKey(1);
}

void drawSpeedGraph(list<int>& speedVector, Mat& img){
	img.setTo(255);

	while (speedVector.size() > img.cols){
		speedVector.pop_front();
	}
	int i = 0;
	for (auto it = speedVector.begin(); it != speedVector.end(); it++){
		int speed = (int)*it * 10;
		line(img, Point(i, img.rows - 1), Point(i, img.rows - 1 - speed), Scalar(0, 0, 255));
		i++;
	}
	for (int i = 0; i < img.rows; i += 100){
		line(img, Point(0, img.rows - 1 - i), Point(img.cols, img.rows - 1 - i), Scalar(100, 100, 100));
	}
}

void drawRotationGraph(vector<double>& rVector, Mat& img, Scalar color){

	if (rVector.size() < 2) return;

	int sIdx = max(0, (int)(rVector.size() - 1 - img.cols));
	int eIdx = rVector.size() - 1;

	int center = img.rows / 2;
	int i = 0;

	int preRot = rVector[sIdx];
	for (int i = sIdx + 1; i <= eIdx; ++i){
		int rot = rVector[i] * 10;
		line(img, Point(i - 1 - sIdx, center - preRot), Point(i - sIdx, center - rot), color);
		preRot = rot;
	}
	for (int i = 0; i < img.rows; i += 100){

		if (i == 0){
			line(img, Point(0, center), Point(img.cols, center), Scalar(0, 0, 0));
		}
		else{

			line(img, Point(0, center - i), Point(img.cols, center - i), Scalar(100, 100, 100));
			line(img, Point(0, center + i), Point(img.cols, center + i), Scalar(100, 100, 100));
		}
	}
}

void skewSymmetric(double a, double b, double c, double* dst)
{
	dst[0] = 0;      dst[1] = -c;   dst[2] = b;
	dst[3] = c;      dst[4] = 0;      dst[5] = -a;
	dst[6] = -b;   dst[7] = a;      dst[8] = 0;
}

void rodriguesVec2Mat(double e1, double e2, double e3, Mat &rotation)
{
	double theta;
	double alpha, beta, gamma;
	int i;
	double omega[3], omegav[9], omegaT[3], tmp[9];// , I[9];

	rotation = Mat::eye(3, 3, DataType<double>::type);

	//rotation[1] = rotation[2] = rotation[3] = rotation[5] = rotation[6] = rotation[7] = 0;
	//rotation[0] = rotation[4] = rotation[8] = 1;

	//I[1] = I[2] = I[3] = I[5] = I[6] = I[7] = 0;
	//I[0] = I[4] = I[8] = 1;

	theta = sqrt(e1*e1 + e2*e2 + e3*e3);
	if (theta > 1e-10){
		omega[0] = e1 / theta;
		omega[1] = e2 / theta;
		omega[2] = e3 / theta;

		alpha = cos(theta);
		beta = sin(theta);
		gamma = 1 - cos(theta);

		skewSymmetric(omega[0], omega[1], omega[2], omegav);
		Mat v = Mat(3, 3, DataType<double>::type, omegav);
		Mat I = Mat::eye(3, 3, DataType<double>::type);
		Mat o = Mat(3, 1, DataType<double>::type, omega);
		Mat t = Mat(1, 3, DataType<double>::type, omega);
		//matTranspose(omega, 3, 1, omegaT);
		Mat m = o*t;
		//matMul(omega, 3, 1, omegaT, 1, 3, tmp);

		//for (i = 0; i < 9; ++i)
		//	rotation[i] = alpha*I[i] + beta*omegav[i] + gamma*tmp[i];
		rotation = alpha*I + beta*v + gamma*t;
	}
}

void rodriguesMat2Vec(double* rotation, double* dst)
{
	double tr, theta, vth;

	tr = (rotation[0] + rotation[4] + rotation[8] - 1)*0.5;
	theta = acos(tr);
	vth = 1. / (2.*sin(theta));

	dst[0] = (rotation[7] - rotation[5])*vth*theta;
	dst[1] = (rotation[2] - rotation[6])*vth*theta;
	dst[2] = (rotation[3] - rotation[1])*vth*theta;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
void rotationMatrixToEulerAngles(Mat &R, Mat& theta)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}

	theta.at<double>(0) = x;
	theta.at<double>(1) = y;
	theta.at<double>(2) = z;
	//return Vec3f(x, y, z);
}

void  eulerAnglesToRotationMatrix(Mat &theta, Mat& R)
{
	// Calculate rotation about x axis
	double rx, ry, rz;
	rx = theta.at<double>(0);
	ry = theta.at<double>(1);
	rz = theta.at<double>(2);

	Mat R_x = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(rx), -sin(rx),
		0, sin(rx), cos(rx)
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3, 3) <<
		cos(ry), 0, sin(ry),
		0, 1, 0,
		-sin(ry), 0, cos(ry)
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3, 3) <<
		cos(rz), -sin(rz), 0,
		sin(rz), cos(rz), 0,
		0, 0, 1);


	// Combined rotation matrix
	R = R_z * R_y * R_x;

}




void predictKalman(Mat &X, Mat &P, double theta, double scale){

	X.at<double>(2) = X.at<double>(2) + theta;
	X.at<double>(0) = X.at<double>(0) + scale*cos(X.at<double>(2));//z
	X.at<double>(1) = X.at<double>(1) + scale*sin(X.at<double>(2));//x

	Mat F = Mat::eye(3, 3, DataType<double>::type);
	F.at<double>(0, 2) = -sin(X.at<double>(2));
	F.at<double>(1, 2) = cos(X.at<double>(2));

	Mat Q = Mat::eye(3, 3, DataType<double>::type);
	Q.at<double>(0, 0) = 0.5*0.5;
	Q.at<double>(1, 1) = 0.5*0.5;
	Q.at<double>(2, 2) = 2.*A2R*2.*A2R;

	P = F*P*F.t() + Q;
}

void updateKalman(Mat &gps, Mat &X, Mat &P, double scale){

	Mat I3x3 = Mat::eye(3, 3, DataType<double>::type);
	Mat residual = gps - X;
	Mat R = Mat::eye(3, 3, DataType<double>::type);
	R.at<double>(0, 0) = 0.2*0.2;
	R.at<double>(1, 1) = 0.2*0.2;
	double rad = atan2(scale, 0.4);
	//std::cout << rad*R2A << std::endl;
	R.at<double>(2, 2) = rad*rad;
	Mat S = P + R;
	Mat K = P*S.inv();
	X = X + K*residual;
	P = (I3x3 - K)*P;
}

int main(){
	int TRAJ_START_X = 300;
	int TRAJ_START_Y = 300;

	Mat covP = Mat::eye(3, 3, DataType<double>::type);
	Mat kalmanState = Mat::zeros(3, 1, DataType<double>::type);
	covP = INF*covP;
	bool initKalman = false;

	string gpsDir = cv::format("%sGPS.txt", DIR);
	string camDir = cv::format("%sCamera_0.txt", DIR);
	string lidarDir = cv::format("%sLidar_", DIR);
	string vehSpeedDir = cv::format("%sVehInfo.txt", DIR);

	vector<LogData> log;
	loadGpsLog(log, gpsDir);
	loadCamLog(log, camDir);
	loadLidarLog(log, lidarDir, 3);
	loadVehSpeedLog(log, vehSpeedDir);


	namedWindow("lidar");
	moveWindow("lidar", 100, 100);
	Mat Lidar = cv::Mat(600, 200, cv::DataType<ColorT>::type);
	sort(log.begin(), log.end(), [](LogData&l, LogData&r){
		return l.timeStamp < r.timeStamp;
	});

	printf("SORT END");

	bool gpsInit = false;
	double initTmX, initTmY, preAltitude;
	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	traj.setTo(255);
	Mat vehGraph = Mat::zeros(500, 800, CV_8UC3);
	list<int> speedVector;

	Mat rotationGraph = Mat::zeros(500, 1200, CV_8UC3);
	vector<double> rxVector;
	vector<double> ryVector;
	vector<double> rzVector;


	////////////////////////////////////////////////////////////////////
	/////////////////////////  Visual Odometry /////////////////////////
	////////////////////////////////////////////////////////////////////

	Mat R_f, t_f; //the final rotation and tranlation vectors containing the
	cv::Rect ROI_RECT(ROI_START_X, ROI_START_Y, ROI_COLS, ROI_ROWS);

	double scale = 1.00;
	//char filename[100];
	std::string filename;// = cv::format("%simage\\camera0_%d%s", DIR, SKIP_FRAME, IMG_TYPE);
	//sprintf(filename, "%simage\\camera0_%d%s", DIR, 0, IMG_TYPE);


	//Mat img_1_c = imread(filename);
	//if (img_1_c.empty()){
	//	return 0;
	//}
	//img_1_c = img_1_c(ROI_RECT).clone();

	//cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);

	Mat prevImage;// = img_1;
	Mat currImage, R, t;

	R_f = Mat::eye(3, 3, DataType<double>::type);
	t_f = Mat::zeros(3, 1, DataType<double>::type);

	clock_t begin = clock();

	namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
	namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.

	VO visualOdometry;
	visualOdometry.init(FOCAL_LENGTH_X, CU - ROI_START_X, CV - ROI_START_Y);
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	int prevSpeedIdx = -1, currSpeedIdx = -1;
	int prevImageIdx = -1;
	bool initCam = false;
	double gpsHeading = 0;
	double voHeading = 0;
	////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////  Lidar  Pointing  ///////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	Mat localPoint;
	bool isChangeLP = false;
	int gpsIdx;
	int preGpsIdx;
	bool isUseGps = true;

	FILE *pointFile;
	pointFile = fopen(format("%spoint.txt", DIR).c_str(), "w");
	if (pointFile == NULL){
		printf("File is not Opened\n");
		return 0;
	}
	printf("log.size() : %d\n", log.size());

	bool sequenceLidar = false;
	for (int i = 0; i < log.size(); i++){
		if (i % 1000 == 0){
			printf("%d\n", i);
		}
		//log[i].gpsData.print();
		//printf("%d %d\n", log[i].frameNum, log[i].timeStamp);
		LogData &logData = log[i];
		//int temp = 3312089;
		//if (logData.timeStamp < temp){
		//	continue;
		//}


		if (logData.dataType == LOG_DATA_TYPE::LIDAR){
			//if (isUseGps==false){
			//	double param[] = { -9.4, 0.23, 0.7, 0.2, -0.5, 0.07 };
			//	//double param[] = { -3.7, -0.3, -0.1, -0.21, -0.4, 0.07 };
			//	vector<int> & lidarData = logData.lidarData[0];
			//	cv::Mat Rx, Ry, Rz;
			//
			//	Rx = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			//	Ry = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			//	Rz = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			//	double *ptr = param;
			//
			//	double A2R = M_PI / 180.0;
			//	double radianX = (*ptr++)*A2R;
			//	double radianY = (*ptr++)*A2R;
			//	double radianZ = (*ptr++)*A2R;
			//
			//
			//	//double radianX = rx*A2R;
			//double radianY = ry*A2R;
			//double radianZ = rz*A2R;
			//
			//	Rx.at<double>(1, 1) = cos(radianX);
			//	Rx.at<double>(1, 2) = -sin(radianX);
			//	Rx.at<double>(2, 1) = sin(radianX);
			//	Rx.at<double>(2, 2) = cos(radianX);
			//
			//	Ry.at<double>(0, 0) = cos(radianY);
			//	Ry.at<double>(0, 2) = sin(radianY);
			//	Ry.at<double>(2, 0) = -sin(radianY);
			//	Ry.at<double>(2, 2) = cos(radianY);
			//
			//	Rz.at<double>(0, 0) = cos(radianZ);
			//	Rz.at<double>(0, 1) = -sin(radianZ);
			//	Rz.at<double>(1, 0) = sin(radianZ);
			//	Rz.at<double>(1, 1) = cos(radianZ);
			//
			//	cv::Mat Rot = Rx*Ry*Rz;
			//	cv::Mat Pc, Pl, T, K, temp, Rgps, Tgps;
			//	T = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
			//	Pl = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
			//	Rgps = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			//	Tgps = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
			//	Rgps.at<double>(0, 0) = cos((gpsHeading - 90)*A2R);
			//	Rgps.at<double>(0, 2) = sin((gpsHeading - 90)*A2R);
			//	Rgps.at<double>(2, 0) = -sin((gpsHeading - 90)*A2R);
			//	Rgps.at<double>(2, 2) = cos((gpsHeading - 90)*A2R);
			//	Tgps.at<double>(0, 0) = log[gpsIdx].gpsData.tmX;
			//	Tgps.at<double>(1, 0) = 0;
			//	Tgps.at<double>(2, 0) = log[gpsIdx].gpsData.tmY;
			//	T.at<double>(0, 0) = (*ptr++);
			//	T.at<double>(1, 0) = (*ptr++);
			//	T.at<double>(2, 0) = (*ptr++);
			//
			//	for (int i = 0; i < lidarData.size(); i++){
			//		const double dRadian = 180.0 / (lidarData.size() - 1) * A2R;
			//		double radi = dRadian * i;
			//		if (lidarData[i] >= 8000){
			//			continue;
			//		}
			//		double lidarZ = sin(radi)*(lidarData[i] / 100.);
			//		double lidarX = cos(radi)*(lidarData[i] / 100.);
			//
			//		Pl.at<double>(2, 0) = lidarZ;
			//		Pl.at<double>(0, 0) = lidarX;
			//		Pc = Rot*Pl + T;
			//		Mat P = Rgps*Pc + Tgps;
			//		//Mat P = R_f.t()*(Pc - t_f);
			//
			//
			//		fprintf(pointFile, "%lf %lf %lf ", P.at<double>(0, 0), P.at<double>(1, 0), P.at<double>(2, 0));
			//		//int u = temp.at<double>(0, 0) / temp.at<double>(2, 0);
			//int v = temp.at<double>(1, 0) / temp.at<double>(2, 0);
			//	}
			//	//fprintf(pointFile, "\n");
			//	isUseGps = true;
			//}
			//continue;
			if (isChangeLP || sequenceLidar){
				double param[2][6] = { { -9.4, 0.23, 0.7, 0.2, -0.5, 0.07 }, { -3.7, -0.3, -0.1, -0.21, -0.4, 0.07 } };
				for (int Li = 0; Li < 1; Li++){

					vector<int> & lidarData = logData.lidarData[0];
					cv::Mat Rx, Ry, Rz;

					Rx = cv::Mat::eye(3, 3, cv::DataType<double>::type);
					Ry = cv::Mat::eye(3, 3, cv::DataType<double>::type);
					Rz = cv::Mat::eye(3, 3, cv::DataType<double>::type);
					double *ptr = param[Li];

					double A2R = M_PI / 180.0;
					double radianX = (*ptr++)*A2R;
					double radianY = (*ptr++)*A2R;
					double radianZ = (*ptr++)*A2R;


					//double radianX = rx*A2R;
					//double radianY = ry*A2R;
					//double radianZ = rz*A2R;

					Rx.at<double>(1, 1) = cos(radianX);
					Rx.at<double>(1, 2) = -sin(radianX);
					Rx.at<double>(2, 1) = sin(radianX);
					Rx.at<double>(2, 2) = cos(radianX);

					Ry.at<double>(0, 0) = cos(radianY);
					Ry.at<double>(0, 2) = sin(radianY);
					Ry.at<double>(2, 0) = -sin(radianY);
					Ry.at<double>(2, 2) = cos(radianY);

					Rz.at<double>(0, 0) = cos(radianZ);
					Rz.at<double>(0, 1) = -sin(radianZ);
					Rz.at<double>(1, 0) = sin(radianZ);
					Rz.at<double>(1, 1) = cos(radianZ);

					cv::Mat Rot = Rx*Ry*Rz;
					cv::Mat Pc, Pl, T, K, temp;
					T = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
					Pl = cv::Mat::zeros(3, 1, cv::DataType<double>::type);

					T.at<double>(0, 0) = (*ptr++);
					T.at<double>(1, 0) = (*ptr++);
					T.at<double>(2, 0) = (*ptr++);

					for (int i = 0; i < lidarData.size(); i++){
						const double dRadian = 180.0 / (lidarData.size() - 1) * A2R;
						double radi = dRadian * i;
						if (lidarData[i] >= 8000){
							continue;
						}
						double lidarZ = sin(radi)*(lidarData[i] / 100.);
						double lidarX = cos(radi)*(lidarData[i] / 100.);

						Pl.at<double>(2, 0) = lidarZ;
						Pl.at<double>(0, 0) = lidarX;
						Pc = Rot*Pl + T;
						Mat P = R_f*Pc + t_f;
						//Mat P = R_f.t()*(Pc - t_f);


						fprintf(pointFile, "%lf %lf %lf ", P.at<double>(0, 0), P.at<double>(1, 0), P.at<double>(2, 0));
						//int u = temp.at<double>(0, 0) / temp.at<double>(2, 0);
						//int v = temp.at<double>(1, 0) / temp.at<double>(2, 0);
					}
				}
				//fprintf(pointFile, "\n");
				isChangeLP = false;
				sequenceLidar = true;
			}
			//showLidarMap(logData.lidarData, Lidar);
			//imshow("lidar", Lidar);
			//waitKey(1);
			//printf("%d %d\n", log[i].timeStamp, log[i].dataType);
		}
		else if (logData.dataType == LOG_DATA_TYPE::CAMERA){
			string imgPath = format("%simage\\camera0_%d%s", DIR, logData.frameNum, IMG_TYPE);
			cv::Mat img = imread(imgPath);
			img = img(ROI_RECT).clone();

			if (initCam == false && prevSpeedIdx != -1){
				initCam = true;
				cvtColor(img, prevImage, COLOR_BGR2GRAY);
				prevImageIdx = i;
			}
			else if (initCam == true){
				cvtColor(img, currImage, COLOR_BGR2GRAY);
				//double dt = logData.timeStamp - log[prevImageIdx].timeStamp;
				double dt = logData.timeStamp - log[prevSpeedIdx].timeStamp;
				double sv = (log[prevSpeedIdx].speed + log[currSpeedIdx].speed) / 3600.;



				scale += sv * dt / 2.;
				if (scale < 0.2){
					prevImageIdx = i;
					prevSpeedIdx = currSpeedIdx;

					continue;
				}

				vector<Point2f> prevFeatures = visualOdometry.prevFeatures;

				bool bCalc = visualOdometry.calcMotion(prevImage, currImage, R, t, scale);



				if (scale < 1.5 &&bCalc == false){// fabs((rodR.at<double>(0) + rodR.at<double>(1) + rodR.at<double>(2))*R2A) < 2){
					prevImageIdx = i;
					prevSpeedIdx = currSpeedIdx;
					visualOdometry.prevFeatures = prevFeatures;

					continue;
				}

				Mat rodR = Mat(3, 1, DataType<double>::type);
				Rodrigues(R, rodR);
				//rotationMatrixToEulerAngles(R, rodR);
				rodR.at<double>(2) = 0;
				rodR.at<double>(0) = 0;
				//eulerAnglesToRotationMatrix(rodR, R);
				Rodrigues(rodR, R);
				t.at<double>(1) = 0;

				if (rxVector.size() == 0){
					rxVector.push_back(rodR.at<double>(0)*R2A);
					ryVector.push_back(rodR.at<double>(1)*R2A);
					rzVector.push_back(rodR.at<double>(2)*R2A);
				}
				else{
					rxVector.push_back(rxVector[rxVector.size() - 1] + rodR.at<double>(0)*R2A);
					ryVector.push_back(ryVector[ryVector.size() - 1] + rodR.at<double>(1)*R2A);
					rzVector.push_back(rzVector[rzVector.size() - 1] + rodR.at<double>(2)*R2A);
					rotationGraph.setTo(255);
					//drawRotationGraph(rxVector, rotationGraph, Scalar(255, 0, 0));
					//drawRotationGraph(ryVector, rotationGraph, Scalar(0, 255, 0));
					//drawRotationGraph(rzVector, rotationGraph, Scalar(0, 0, 255));
					//std::cout << ryVector[ryVector.size() - 1] << std::endl;
				}


				if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
					//t_f = t_f + scale*(R_f*t);
					//R_f = R*R_f;
					Mat tmpAng = cv::Mat::zeros(3, 1, DataType<double>::type);
					//rotationMatrixToEulerAngles(R_f, tmpAng);
					Rodrigues(R_f, tmpAng);
					tmpAng.at<double>(1) += rodR.at<double>(1);
					//eulerAnglesToRotationMatrix(tmpAng, R_f);
					Rodrigues(tmpAng, R_f);
					t_f.at<double>(0) += scale * sin(tmpAng.at<double>(1));
					t_f.at<double>(2) += scale * cos(tmpAng.at<double>(1));

					if (initKalman){
						predictKalman(kalmanState, covP, rodR.at<double>(1), scale);
						tmpAng.at<double>(1) = kalmanState.at<double>(2);
						Rodrigues(tmpAng, R_f);
						t_f.at<double>(0) = kalmanState.at<double>(1);
						t_f.at<double>(2) = kalmanState.at<double>(0);
					}

				}
				else {
					//cout << "scale below 0.1, or incorrect translation" << endl;
				}
				//rotationMatrixToEulerAngles(R_f, rodR);
				Rodrigues(R_f, rodR);
				//rodR.at<double>(2) = 0;
				//Rodrigues(rodR, R_f);
				voHeading = rodR.at<double>(1)*R2A;
				//cout << R << endl;
				//cout << R2A*rodR << endl;


				//cv::imshow("rotationGraph", rotationGraph);

				for (int i = 0; i < visualOdometry.prevFeatures.size(); i++){
					line(img, visualOdometry.prevFeatures[i], visualOdometry.currFeatures[i], Scalar(0, 255, 0));
					circle(img, visualOdometry.currFeatures[i], 2, Scalar(0, 255, 0), -1);
				}
				scale = 0;
				scale = -sv * dt / 2.;
				visualOdometry.updateFeatures();
				prevImage = currImage.clone();
				prevSpeedIdx = currSpeedIdx;
				prevImageIdx = i;
				int x = int(t_f.at<double>(0)) + TRAJ_START_X;
				int y = int(t_f.at<double>(2)) + TRAJ_START_Y;
				circle(traj, Point(x, y), 1, Scalar(255, 0, 0), 2);
				//printf("%d %d\n", x, y);

				//rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
				//putText(traj, format("%.0f    %.0f    %.0f", voHeading, gpsHeading, gpsHeading - voHeading), Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar::all(255));
				//sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
				//putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
				//sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

				localPoint = Mat(3, 1, DataType<double>::type, { t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2) });
				//fprintf(pointFile, "%lf %lf %lf ", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

				isChangeLP = true;


				imshow("Road facing camera", img);
				imshow("Trajectory", traj);

				waitKey(1);
			}
		}
		else if (logData.dataType == LOG_DATA_TYPE::GPS){

			if (initCam == true && !gpsInit&&logData.gpsData.gpsDataType == GPS_DATA_TYPE::GPGGA){
				preGpsIdx = gpsIdx = i;
				isUseGps = false;
				initTmX = logData.gpsData.tmX;
				initTmY = logData.gpsData.tmY;
				preAltitude = logData.gpsData.altitude;
				gpsHeading = logData.gpsData.heading;
				cv::Mat tmp = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
				tmp.at<double>(1) = (gpsHeading - 90)*A2R;

				Rodrigues(tmp, R_f);
				t_f = cv::Mat::zeros(3, 1, DataType<double>::type);

				initKalman = true;
				kalmanState.at<double>(0) = 0;
				kalmanState.at<double>(1) = 0;
				kalmanState.at<double>(2) = tmp.at<double>(1);
				gpsInit = true;
			}
			if (gpsInit && logData.gpsData.gpsDataType == GPS_DATA_TYPE::GPGGA){
				gpsIdx = i;
				isUseGps = false;
				gpsHeading = logData.gpsData.heading;
				int x = logData.gpsData.tmX - initTmX + TRAJ_START_X;
				int y = logData.gpsData.tmY - initTmY + TRAJ_START_Y;
				//rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
				//cv::putText(traj, format("%d    %d     %d", x, y, logData.timeStamp), Point(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0, 0, 255));
				//putText(traj, format("%.0f    %.0f    %.0f", voHeading, gpsHeading, gpsHeading - voHeading), Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar::all(255));
				circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
				cv::imshow("Trajectory", traj);

				cv::Mat gpsState = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
				gpsState.at<double>(1) = logData.gpsData.tmX - initTmX;
				gpsState.at<double>(0) = logData.gpsData.tmY - initTmY;
				gpsState.at<double>(2) = (gpsHeading - 90)*A2R;
				//scale = 
				updateKalman(gpsState, kalmanState, covP, scale);

				waitKey(1);
			}
		}
		else if (logData.dataType == LOG_DATA_TYPE::VEH_SPEED){
			if (initCam == false){
				prevSpeedIdx = i;
			}
			else{
				currSpeedIdx = i;
				double dt = logData.timeStamp - log[prevSpeedIdx].timeStamp;
				double sv = (log[prevSpeedIdx].speed + logData.speed) / 3600.;

				scale += sv * dt / 2.;
				prevSpeedIdx = currSpeedIdx;
			}
			//speedVector.push_back(logData.speed);
			//drawSpeedGraph(speedVector, vehGraph);
			//cv::imshow("vehGraph", vehGraph);
			//waitKey(1);
		}

		//if (i > 0){
		//	printf("%d\n", log[i].timeStamp - log[i-1].timeStamp);
		//}

		//if (i % 100 == 0){
		//	getchar();
		//}
	}
	std::string Trajectory;
	std::stringstream sstream;
	sstream << format("%sTrajectory.bmp", DIR);
	sstream >> Trajectory;
	cv::imwrite(Trajectory, traj);
	waitKey();



	return 0;
}