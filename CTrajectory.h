#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H
//	Undefined time stamp
#define SEC_UNDEFINED	0xFFFFFFFF


#include <fstream>
#include <string>
#include "COrientinfor.h"
#include<stdbool.h>
#include<atlstr.h>
using std::string;
using std::fstream;



typedef unsigned char BYTE;
typedef struct tag_Trajectory
{
	double t;
	double lat;//角度单位是弧度
	double lon;
	double alt;
	double vx;
	double vy;
	double vz;
	double roll;
	double pitch;
	double head;
	double wander;
	double bax;
	double bay;
	double baz;
	double ax;
	double ay;
	double az;
}Trajectory;


typedef struct{
	double x;
	double y;
	double z;
}Dp3d;



typedef struct{
	double Time ; // Time stamp (seconds in some system)
	Dp3d Xyz ; // Position
	double Head ; // Heading (degrees)
	double Roll ; // Roll (degrees)
	double Pitch ; // Pitch (degrees)
	int Quality ; // Quality tag value (0-5)
	int Mark ; // Run time flag
}TrajPos;



typedef struct {
	char Recog[8]; // TSCANTRJ
	int Version; // File version 20010715
	int HdrSize; // sizeof(TrajHdr)
	int PosCnt; // Number of position records
	int PosSize; // Size of position records
	char Desc[79]; // Description
	BYTE Quality; // Quality for whole trajectory (1-5)
	double BegTime; // First time stamp
	double EndTime; // Last time stamp
	int OrigNbr; // Original number (before any splitting)
	int Number; // Flightline number (in laser points)
	char VrtVideo[400]; // Vertical facing video
	double VrtBeg; // Start time of VrtVideo[]
	double VrtEnd; // End time of VrtVideo[]
	char FwdVideo[400]; // Forward facing video
	double FwdBeg; // Start time of FwdVideo[]
	double FwdEnd; // End time of FwdVideo[]
} TrajHdr;

typedef struct tag_POSStatus
{
	//角度偏移
	double m_dROffset;
	double m_dPOffset;
	double m_dHOffset;
	//角度漂移
	double m_dRDrift;
	double m_dPDrift;
	double m_dHDrift;
	//位置偏移
	double m_dXOffset;
	double m_dYOffset;
	double m_dZOffset;
	//位置漂移
	double m_dXDrift;
	double m_dYDrift;
	double m_dZDrift;
}POSStatus;

typedef struct tag_LdrErrorSrc
{
	//姿态与位置误差
	double m_dRError;
	double m_dPError;
	double m_dHError;
	double m_dXError;
	double m_dYError;
	double m_dZError;
	//扫描角误差
	double m_dAngError;//扫描角度误差偏移
	double m_dAngSclError;//扫描角线性漂移误差，实际上是扭矩造成的
	//测距误差
	double m_dRangeError;
	//时间误差
	double m_dtError;
}LdrErrorSrc;


/*
 *	本类作为轨道和姿态的基类，这样可以作为外面的统一接口进行访问
 */
class CTrajectory
{
public:
	CTrajectory(){PI = 3.1415926;};
	~CTrajectory(){};
public:
	virtual bool Open(string filename)=0;
	virtual long GetRecordNum()=0;
	virtual bool GetNext(OrientInfo& orient)=0;
	virtual bool GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient)=0;
	virtual void Close()=0;
	//获取轨迹文件的类型，目前只支持两种格式文件：SBET和TERRASOLID的TRJ格式
//	virtual  TrjType GetTrajectoryType()=0;
	virtual  int GetTrajectoryType()=0;
protected:
	long m_lBuffSz;
	double PI;
};
/**********************************************************
*	本类用来获取SBET格式数据，主要是轨道数据和姿态信息
 **********************************************************/
 class __declspec(dllexport)CPosPacTrj:public CTrajectory
{
public:
	CPosPacTrj();
	~CPosPacTrj();
public:
	bool Open(string filename);
	long GetRecordNum();
	bool GetNext(OrientInfo& orient);
	bool GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient);
	bool GetNext(Trajectory& trace);
	bool GetRecord(int nPos,int nNum,Trajectory* pTrace);
	void Close();
//	TrjType  GetTrajectoryType(){return TRJ_SBET;};
	int GetTrajectoryType(){return 0;};
	//添加数据噪声
	BOOL AddNoise(POSStatus Noise,CString strOut);
	//快速定位至指定时间的位置，为提供轨道信息和姿态定位新信息内插服务
	BOOL Goto(double t);
	Trajectory* GetOrientInfor(double t1,double t2);
private:
	long m_nRecord;
	long m_nCurPos;
	bool m_bOpen;
	fstream m_fstream;
	int m_nGotoStart;
};

 /*
  *	本格式角度都是度数 
  */
class __declspec(dllexport)CTScanTrj:public CTrajectory
{
public:
	CTScanTrj();
	~CTScanTrj();
public:
	bool Open(string filename);
	long GetRecordNum();
	bool GetNext(OrientInfo& orient);
	bool GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient);
	bool GetNext(TrajPos& trj);
	bool GetRecord(int nPos,int nNum,TrajPos* pTrj);
	void Close();
//	TrjType GetTrajectoryType(){ return TRJ_TERRA;};
	int GetTrajectoryType(){return 1;};
	TrajPos* GetRecord(double t);
	TrajPos* GetFastRecord(double t);
	TrajPos* GetRecord(){return m_pTrj;}
	//add a new interface for the calibration to determine the exact position of the trj file
	bool Open(string filename,double dtMin,double dtMax);
	//newly added for the dtermination of the unmatched LiDAR file with the trajectory
	BOOL Open(string filename,string strLiDARFile,double dtMin,double dtMax);
	BOOL GetRecord(TrajPos& trj,double t);
//private:
	long m_nRecord;
	long m_nCurPos;
	bool m_bOpen;
	TrajHdr m_TrajHdr;
	fstream m_fstream;
	TrajPos* m_pTrj;
	int m_nCurRecord;
};

__declspec(dllexport) BOOL SBET2Trj(CString strSBET,CString strTrj,double dLonCenter);
#endif
