#ifndef GPSPASER_H
#define GPSPASER_H
#include <string>
typedef struct GPGGA{
	GPGGA(){}
	double latitude;		// ����
	double longitude;		// �浵
	int dataQual;			// ���Ż���
	double altitude;		// ��		���� ����
}GPGGA;

typedef struct GPRMC{
	GPRMC(){}
	int dataQual;			// ���Ż���
	double latitude;		// ����
	double longitude;		// �浵
	double speed;			// �ӷ�
	double heading;			// ���Ⱒ
}GPRMC;

enum class GPS_DATA_TYPE{ GPRMC, GPGGA };
typedef struct GpsData{
	int dataQual;		// 1: gps ����, 2 : DGPS ����
	GPS_DATA_TYPE gpsDataType;
	double tmX;
	double tmY;
	double speed;		// knots ����
	double heading;		// degree ����
	double latitude;		// ����
	double longitude;		// �浵
	double altitude;		// ��
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
		printf("���Ż��� :				%d\n", dataQual);
		printf("TM��ǥ x�� (���� +) :	%lf\n", tmX);
		printf("TM��ǥ y�� (���� +) :	%lf\n", tmY);
		printf("�ӵ� :					%lf\n", speed);
		printf("���Ⱒ :				%lf\n", heading);
		printf("���� :					%lf\n", latitude);
		printf("�浵 :					%lf\n", longitude);
		printf("�� :					%lf\n", altitude);
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