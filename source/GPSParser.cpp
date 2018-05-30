#include "GPSParser.h"


extern void Bessel2TM (double alt, double lon, double lat, double lonOrg, double latOrg, double& tm_x, double& tm_y);

CGPSParser::CGPSParser(void)
{
	gps.altitude = 0;
	gps.dataQual = 0;
	gps.heading = 0;
	gps.speed = 0 ;
	gps.tmX = 0;
	gps.tmY = 0;
}


CGPSParser::~CGPSParser(void)
{
}

bool CGPSParser::parser(GpsData &param, std::string str, double altitude)
{
	if (str.find("$GPGGA") != std::string::npos){
		return processGPGGA(param, str);
	}
	else if (str.find("$GPRMC") != std::string::npos){
		return processGPRMC(param, str, altitude);
	}
	return false;
}

bool CGPSParser::processGPGGA(GpsData &param, std::string substr)
{
	//if (str.find("$GPGGA") == std::string::npos){
	//	return false;
	//}
	GPGGA _GPGGA;
	
	std::string tstr;
	int endComma, startComma;
	
	endComma = substr.find(",");
	startComma = endComma;		// 타입 점프

	endComma = substr.find(",", startComma+1);
	startComma = endComma;		//	위성 시간 점프

	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPGGA.latitude = atof(tstr.c_str());		//	위도

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		//	N

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPGGA.longitude = atof(tstr.c_str());		//	경도

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		//	E

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPGGA.dataQual = atoi(tstr.c_str());		//	데이터 상태

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		// 위성 개수

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		// 데이터 질

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPGGA.altitude = atof(tstr.c_str());		// 고도
	
	if(_GPGGA.dataQual > 0)
	{
		Bessel2TM(_GPGGA.altitude, _GPGGA.longitude, _GPGGA.latitude, 127, 38, gps.tmX, gps.tmY);
		gps.dataQual = _GPGGA.dataQual;
		gps.altitude = _GPGGA.altitude;
		gps.longitude = _GPGGA.longitude;
		gps.latitude = _GPGGA.latitude;
		gps.gpsDataType = GPS_DATA_TYPE::GPGGA;
		param = gps;
	}

	return true;
	//startlen = str.find("$", endlen);
	//if(startlen == -1)
	//	return false;
	//endlen = str.find("\r\n", startlen);
}

bool CGPSParser::processGPRMC(GpsData &param, std::string substr, double altitude)
{
	//if (str.find("$GPRMC")==std::string::npos){
	//	return false;
	//}
	GPRMC _GPRMC;
	int startlen, endlen;
	std::string tstr;
	int endComma, startComma;

	endComma = substr.find(",");
	startComma = endComma;		// 타입 점프

	endComma = substr.find(",", startComma+1);
	startComma = endComma;		//	위성 시간 점프

	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	startComma = endComma;

	if(tstr == "A")
		_GPRMC.dataQual = 1;			//	데이터 상태
	else
		_GPRMC.dataQual = 0;			//	데이터 상태

	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPRMC.latitude = atoi(tstr.c_str());		//	위도

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		//	N

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPRMC.longitude = atof(tstr.c_str());		//	경도

	startComma = endComma;
	endComma = substr.find(",", startComma+1);		//	W

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPRMC.speed = atof(tstr.c_str());		//	speed

	startComma = endComma;
	endComma = substr.find(",", startComma+1);
	tstr = substr.substr(startComma+1, endComma-startComma-1);
	_GPRMC.heading = atof(tstr.c_str());		// heading
	
	if(_GPRMC.dataQual == 1)
	{
		//_Bessel2TM(_GPRMC.longitude, _GPRMC.latitude, 127, 38, gps.tmX, gps.tmY);
		Bessel2TM(altitude, _GPRMC.longitude, _GPRMC.latitude, 127, 38, gps.tmX, gps.tmY);

		gps.dataQual = _GPRMC.dataQual;
		gps.heading = _GPRMC.heading;
		gps.latitude = _GPRMC.latitude;
		gps.longitude = _GPRMC.longitude;
		gps.gpsDataType = GPS_DATA_TYPE::GPRMC;
		if(gps.heading > 180)
			gps.heading -= 360;
		
		gps.speed = _GPRMC.speed * 1.852;//note
		param = gps;
	}

	return true;
	//std::cout << _GPRMC.heading << std::endl;

	//startlen = str.find("$", endlen);
	//if(startlen == -1)
	//	return false;
	//endlen = str.find("\r\n", startlen);
	//return true;
}