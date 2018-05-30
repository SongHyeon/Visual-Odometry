#ifndef GPSPASER_H
#define GPSPASER_H
#include <string>
typedef struct GPGGA{
	GPGGA(){}
	double latitude;		// 위도
	double longitude;		// 경도
	int dataQual;			// 수신상태
	double altitude;		// 고도		미터 단위
}GPGGA;

typedef struct GPRMC{
	GPRMC(){}
	int dataQual;			// 수신상태
	double latitude;		// 위도
	double longitude;		// 경도
	double speed;			// 속력
	double heading;			// 방향각
}GPRMC;

enum class GPS_DATA_TYPE{ GPRMC, GPGGA };
typedef struct GpsData{
	int dataQual;		// 1: gps 수신, 2 : DGPS 수신
	GPS_DATA_TYPE gpsDataType;
	double tmX;
	double tmY;
	double speed;		// knots 단위
	double heading;		// degree 단위
	double latitude;		// 위도
	double longitude;		// 경도
	double altitude;		// 고도
	GpsData() :
		dataQual(0),
		tmX(0),
		tmY(0),
		speed(0),
		heading(0),
		latitude(0),
		longitude(0),
		altitude(0)
	{}
	void print(){
		printf("수신상태 :				%d\n", dataQual);
		printf("TM좌표 x축 (북쪽 +) :	%lf\n", tmX);
		printf("TM좌표 y축 (동쪽 +) :	%lf\n", tmY);
		printf("속도 :					%lf\n", speed);
		printf("방향각 :				%lf\n", heading);
		printf("위도 :					%lf\n", latitude);
		printf("경도 :					%lf\n", longitude);
		printf("고도 :					%lf\n", altitude);
	}
}GpsData;

class CGPSParser
{
public:
	CGPSParser(void);
	~CGPSParser(void);

	GpsData gps;
	bool parser(GpsData &param, std::string str, double altitude = 0);
	bool processGPGGA(GpsData &param, std::string substr);
	bool processGPRMC(GpsData &param, std::string substr, double altitude);
};

#endif 