#ifndef _ORIENT_INFOR_H_
#define _ORIENT_INFOR_H_
#include <string>
#include "math.h"
using std::string;
class CTrajectory;
typedef struct tag_OrientInfo
{
	double t;
	//角度单位是弧度，默认情况下一般为经纬度，对于trajectory 格式则为相应的高斯投影平面坐标
	//和椭球高或者其他高程系统高程
	double lat;
	double lon;
	double alt;
	double r;//
	double p;
	double h;
public:
	//判断是否是经纬度还是高斯投影加大地高
	bool IsLatLonH(){ return fabs(lat)<7;}
}OrientInfo;

typedef struct tag_NavPosOrient
{
	double t;
	double X;
	double Y;
	double Z;
	double r;
	double p;
	double h;
}NavPosOrient;

class COrientInfor{
public:
	COrientInfor();
	COrientInfor(string filename);
	~COrientInfor();
public:	
	//设置轨迹参数缓存
	void SetBufferSz(long lBuffSz);
	bool Open(string filename);
	long GetRecordNum();
	bool GetNext(OrientInfo& orient);
	//获取到特定时刻的外方位元素
	bool GetOrientInfor(double t,OrientInfo& orient);
	bool GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient);
	void Close();
private:
	CTrajectory* m_pTrj;
};
#endif