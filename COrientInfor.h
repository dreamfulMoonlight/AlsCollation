#ifndef _ORIENT_INFOR_H_
#define _ORIENT_INFOR_H_
#include <string>
#include "math.h"
using std::string;
class CTrajectory;
typedef struct tag_OrientInfo
{
	double t;
	//�Ƕȵ�λ�ǻ��ȣ�Ĭ�������һ��Ϊ��γ�ȣ�����trajectory ��ʽ��Ϊ��Ӧ�ĸ�˹ͶӰƽ������
	//������߻��������߳�ϵͳ�߳�
	double lat;
	double lon;
	double alt;
	double r;//
	double p;
	double h;
public:
	//�ж��Ƿ��Ǿ�γ�Ȼ��Ǹ�˹ͶӰ�Ӵ�ظ�
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
	//���ù켣��������
	void SetBufferSz(long lBuffSz);
	bool Open(string filename);
	long GetRecordNum();
	bool GetNext(OrientInfo& orient);
	//��ȡ���ض�ʱ�̵��ⷽλԪ��
	bool GetOrientInfor(double t,OrientInfo& orient);
	bool GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient);
	void Close();
private:
	CTrajectory* m_pTrj;
};
#endif