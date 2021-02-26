#include "CTrajectory.h"
#include <math.h>
#include<iostream>

#define PID 0.017453292519943295769236907684886
#define DPI 57.295779513082320876798154814105
//const double PI=3.14159265353846;

using namespace std;

CPosPacTrj::CPosPacTrj()
{
	m_nRecord = 0;
	m_nCurPos = 0;
	m_bOpen = false;
	m_nGotoStart = -1;
}



CPosPacTrj::~CPosPacTrj()
{
	Close();
}



bool CPosPacTrj::Open(string filename)
{
	bool bRet = true;
	FILE* m_file = fopen(filename.c_str(),"r");
	if(!m_file)return false;
	fseek(m_file,0,SEEK_END);
	long lSize = ftell(m_file);
	m_nRecord = lSize/136;
	fclose(m_file);
	if(m_fstream.is_open())m_fstream.close();
	//m_fstream.open(filename.c_str(),ios::in|ios::binary,filebuf::sh_read);
	m_fstream.open(filename.c_str(),ios::in|ios::binary);
	m_bOpen = true;	
	return true;
}



long CPosPacTrj::GetRecordNum()
{
	return m_nRecord;
}



bool CPosPacTrj::GetNext(Trajectory& trace)
{
	if(!m_bOpen)return false;
	if(m_nCurPos>=m_nRecord) return false;
	m_fstream.read((char*)&trace,136);
	m_nCurPos++;
	return true;
}



void CPosPacTrj::Close()
{
	if(m_fstream.is_open())
	m_fstream.close();
	m_nRecord = 0;
	m_bOpen = false;
	return;
}

/*
 *	
 */

bool CPosPacTrj::GetRecord(int nPos,int nNum,Trajectory* pTrace)
{
	if(nPos<0) return false;
	if(nPos>=m_nRecord)return false;
	if(nNum<1) return false;
	if((nPos+nNum)>=m_nRecord) return false;
	m_fstream.seekg(136*nPos,ios::beg);
	m_fstream.read(( char*)pTrace,136*nNum);
	m_nCurPos = nPos+nNum;
	return true;
}



bool CPosPacTrj::GetNext(OrientInfo& orient)
{
	Trajectory trace;
	bool bRet = GetNext(trace);
	if(!bRet)return false;
	orient.t = trace.t;
	orient.lat = trace.lat;
	orient.lon = trace.lon;
	orient.alt = trace.alt;
	orient.r = trace.roll;
	orient.p = trace.pitch;
	orient.h = trace.head-trace.wander;
	return true;
}



bool CPosPacTrj::GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient)
{
	Trajectory* pTrace = new Trajectory;
	bool bRet = GetRecord(nPos,nNum,pTrace);
	if(!bRet)
	{
		delete []pTrace;
		pTrace = NULL;
		return false;
	}
	for(int i=0;i<nNum;i++)
	{
		//pOrient[i].t 
		pOrient[i].t = pTrace[i].t;
		pOrient[i].lat = pTrace[i].lat;
		pOrient[i].lon = pTrace[i].lon;
		pOrient[i].alt = pTrace[i].alt;
		pOrient[i].r = pTrace[i].roll;
		pOrient[i].p = pTrace[i].pitch;
		pOrient[i].h = pTrace[i].head-pTrace[i].wander;
	}
	delete []pTrace;
	pTrace = NULL;
	return true;
}



BOOL CPosPacTrj::AddNoise(POSStatus Noise,CString strOut)
{
// 	int nPtNum = GetRecordNum();
// 	//每次读取的点数
// 	int nPtEach = 10000;
// 	Trajectory* pTrace = new Trajectory[nPtEach];
// 	int nInPtNum = nPtNum;
// 	if(!m_bOpen) return m_bOpen;
// 	string strFileOut(strOut.GetBuffer(strOut.GetLength()));
// 	fstream outstream;
// 	outstream.open(strFileOut.c_str(),ios::out|ios::binary,filebuf::sh_write);
// 	m_fstream.seekg(0,ios::beg);
// 	//开始时间
// 	double dStartT = 0.0;
// 	double dt = 0.0;
// 	for(int i=0;i<nPtNum;i+=nPtEach)
// 	{
// 		//本次可读取的点数目
// 		nInPtNum = min(nPtEach,nPtNum-i);
// 		//读取原始轨迹点
// 		m_fstream.read((signed char*)pTrace,136*nInPtNum);
// 		for(int j=0;j<nInPtNum;j++)
// 		{
// 			if(i==0&&j==0)
// 			{
// 				dStartT = pTrace[j].t;
// 			}
// 			//对每个点加噪声
// 			dt = pTrace[j].t-dStartT;
// 			//纬度对应于Y坐标，和高斯投影坐标不同，采用一般的数学坐标系
// 			//定义方式
// 			pTrace[j].lat += Noise.m_dYOffset+Noise.m_dYDrift*dt;
// 			pTrace[j].lon += Noise.m_dXOffset+Noise.m_dXDrift*dt;
// 			pTrace[j].alt += Noise.m_dZOffset+Noise.m_dZDrift*dt;
// 			pTrace[j].roll +=Noise.m_dROffset+Noise.m_dRDrift*dt;
// 			pTrace[j].pitch+=Noise.m_dPOffset+Noise.m_dPDrift*dt;
// 			pTrace[j].head +=Noise.m_dHOffset+Noise.m_dHDrift*dt;
// 		}
// 		outstream.write((signed char*)pTrace,136*nInPtNum);
// 	}
// 	outstream.close();
 	return TRUE;
}



/*
*		//快速定位至指定时间的位置，为提供轨道信息和姿态定位新信息内插服务

 */

BOOL CPosPacTrj::Goto(double t)

{
	int nPos = 0;
	int nNum = m_nRecord/10;
	Trajectory* pTrace = new Trajectory[nNum+1];
	int i = 0;
	int j = 0;
	int nReaded = 0;
	int nInRead = 0;
	BOOL bRet = FALSE;
	for(i=0;i<m_nRecord;i+=nNum)
	{
		m_fstream.seekg(136*nReaded,ios::beg);
		nInRead = min(nReaded+nNum+1,m_nRecord);
		nInRead -= nReaded;
		if(nInRead<=0)
		{
			goto End;
		}
		m_fstream.read(( char*)pTrace,136*nInRead);
		for(j=0;j<nInRead-1;j++)
		{
			if(pTrace[j].t<=t&&pTrace[j+1].t>=t)
			{
				bRet = TRUE;
				nReaded += j;
				goto End;
			}
		}
		nReaded += nInRead-1;
	}
End:
	if(bRet)
	m_nGotoStart = nReaded;
	delete []pTrace;
	pTrace = NULL;
	return bRet;
}



/*
 *	
 */

Trajectory* CPosPacTrj::GetOrientInfor(double t1,double t2)

{
	BOOL bRet = FALSE;
	bRet = TRUE;
	return NULL;
}




//for CTScanTrj
CTScanTrj::CTScanTrj()
{
	m_nRecord = 0;
	m_nCurPos = 0;
	m_bOpen = false;
	m_pTrj = NULL;
	m_nCurRecord = 0;
}



CTScanTrj::~CTScanTrj()
{
	Close();
}



bool CTScanTrj::Open(string filename)
{
	/*
	Close();
		bool bRet = TRUE;
		FILE* m_file = fopen(filename.c_str(),"r");
		if(!m_file)return false;
		char* pBuffer = new char[960];
		char* ptemp = pBuffer;
		fread(pBuffer,1,960,m_file);
		memcpy(m_TrajHdr.Recog,ptemp,8);
		ptemp += 8;
		memcpy(&m_TrajHdr.Version,ptemp,4);
		ptemp += 4;
		memcpy(&m_TrajHdr.HdrSize,ptemp,4);
		ptemp += 4;
		memcpy(&m_TrajHdr.PosCnt,ptemp,4);
		ptemp += 4;
		memcpy(&m_TrajHdr.PosSize,ptemp,4);
		memcpy(m_TrajHdr.Desc,ptemp,79);
		ptemp += 79;
		memcpy(&m_TrajHdr.Quality,ptemp,1);
		ptemp += 1;
		memcpy(&m_TrajHdr.BegTime,ptemp,8);
		ptemp += 8;
		memcpy(&m_TrajHdr.EndTime,ptemp,8);
		ptemp += 8;
		memcpy(&m_TrajHdr.OrigNbr,ptemp,4);
		ptemp += 4;
		memcpy(&m_TrajHdr.Number,ptemp,4);
		ptemp += 4;
		memcpy(m_TrajHdr.VrtVideo,ptemp,400);
		ptemp += 400;
		memcpy(&m_TrajHdr.VrtBeg,ptemp,8);
		ptemp += 8;
		memcpy(&m_TrajHdr.VrtEnd,ptemp,8);
		ptemp += 8;
		memcpy(m_TrajHdr.FwdVideo,ptemp,400);
		ptemp += 400;
		memcpy(&m_TrajHdr.FwdBeg,ptemp,8);
		ptemp += 8;
		memcpy(&m_TrajHdr.FwdEnd,ptemp,8);
		ptemp += 8;
		fseek(m_file,0,SEEK_END);
		long lSize = ftell(m_file);
		m_nRecord = (lSize-960)/64;
		m_nRecord = m_TrajHdr.Number;
		m_nCurPos = 0;
		if(m_fstream.is_open())m_fstream.close();
		m_fstream.open(filename.c_str(),ios::in|ios::binary,filebuf::sh_read);
		m_fstream.seekg(m_TrajHdr.HdrSize,ios::beg);
		m_bOpen = true;
		delete []pBuffer;
		pBuffer = NULL;
	
		if(m_pTrj)delete []m_pTrj;
		m_pTrj = NULL;
		m_pTrj = new TrajPos[m_nRecord];
		m_fstream.read((signed char*)m_pTrj,m_nRecord*64);
		//m_fstream.seekg(960,ios::beg);	
		return true;*/
	Close();
	bool bRet = TRUE;
	FILE* m_file = fopen(filename.c_str(),"r");
	if(!m_file)return false;
	char* pBuffer = new char[960];
	char* ptemp = pBuffer;
	fread(pBuffer,1,960,m_file);
	memcpy(m_TrajHdr.Recog,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.Version,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.HdrSize,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosCnt,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosSize,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.Desc,ptemp,79);
	ptemp += 79;
	memcpy(&m_TrajHdr.Quality,ptemp,1);
	ptemp += 1;
	memcpy(&m_TrajHdr.BegTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.EndTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.OrigNbr,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.Number,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.VrtVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.VrtBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.VrtEnd,ptemp,8);
	ptemp += 8;
	memcpy(m_TrajHdr.FwdVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.FwdBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.FwdEnd,ptemp,8);
	ptemp += 8;
	fseek(m_file,0,SEEK_END);
	long lSize = ftell(m_file);
	fclose(m_file);
	m_nRecord = m_TrajHdr.PosCnt;

    m_nCurPos = 0;
	if(m_fstream.is_open())m_fstream.close();
	//m_fstream.open(filename.c_str(),ios::in|ios::out|ios::binary,filebuf::sh_read|filebuf::sh_write);
	m_fstream.open(filename.c_str(),ios::in|ios::out|ios::binary);
	m_bOpen = true;
	/*
	m_fstream.seekg(960,ios::beg);
	delete []pBuffer;
	pBuffer = NULL;

	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	m_pTrj = new TrajPos[m_nRecord];
	m_fstream.read((signed char*)m_pTrj,m_nRecord*64);*/
	m_fstream.seekg(m_TrajHdr.HdrSize,ios::beg);	
	m_bOpen = true;
	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	m_pTrj = new TrajPos[m_nRecord];
	m_fstream.read(( char*)m_pTrj,m_nRecord*64);
	/*
	m_fstream.seekg(960,ios::beg);
	delete []pBuffer;
	pBuffer = NULL;

	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	m_pTrj = new TrajPos[m_nRecord];
	m_fstream.read((signed char*)m_pTrj,m_nRecord*64);*/
		
	return true;
	
}

//add a new interface for the calibration to determine the exact position of the trj file
bool CTScanTrj::Open(string filename,double dtMin,double dtMax)
{
	Close();
	bool bRet = TRUE;
	FILE* m_file = fopen(filename.c_str(),"r");
	if(!m_file)return false;
	char* pBuffer = new char[960];
	char* ptemp = pBuffer;
	fread(pBuffer,1,960,m_file);
	memcpy(m_TrajHdr.Recog,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.Version,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.HdrSize,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosCnt,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosSize,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.Desc,ptemp,79);
	ptemp += 79;
	memcpy(&m_TrajHdr.Quality,ptemp,1);
	ptemp += 1;
	memcpy(&m_TrajHdr.BegTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.EndTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.OrigNbr,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.Number,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.VrtVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.VrtBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.VrtEnd,ptemp,8);
	ptemp += 8;
	memcpy(m_TrajHdr.FwdVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.FwdBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.FwdEnd,ptemp,8);
	ptemp += 8;
	fseek(m_file,0,SEEK_END);
	long lSize = ftell(m_file);
	fclose(m_file);
	m_nRecord = m_TrajHdr.PosCnt;

    m_nCurPos = 0;
	if(m_fstream.is_open())m_fstream.close();
	m_fstream.open(filename.c_str(),ios::in|ios::out|ios::binary);
	m_bOpen = true;
	//
	m_fstream.seekg(m_TrajHdr.HdrSize,ios::beg);	
	m_bOpen = true;
	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	//读取一部分，判断里面是否包含所需要的记录
	int nReaded = 0;
	int nReadStep = 100000;
	if(m_TrajHdr.BegTime>dtMin||m_TrajHdr.EndTime<dtMax)
	{
		//文件范围里面的数据比所要求的范围小 则以失败返回
		m_fstream.close();
		m_bOpen = false;
		CString strTemp(filename.c_str());
		strTemp="存在时间范围不一致的航迹文件(如"+strTemp+")和航带，请查对!";
		cout << strTemp << endl;
		return false;
	}

	if(nReadStep>=m_nRecord)
	{
		//记录数比较小的简单情形
		m_pTrj = new TrajPos[m_nRecord];
		m_fstream.read(( char*)m_pTrj,m_nRecord*64);
	}
	else
	{
		//记录数比较多的复杂情形
		TrajPos* pTempTrj = NULL;
		TrajPos* pTrjRecord = NULL;
		BOOL bFinded = FALSE;
		BOOL bMinX = FALSE;
		int nIndex = 0;
		int nMaxIndex = 0;
		while(!bMinX&&nReaded<m_nRecord)
		{
			pTempTrj = new TrajPos[nReadStep];
			m_fstream.read(( char*)pTempTrj,nReadStep*64);
			for(int j=0;j<nReadStep;j++)
			{
				if((pTempTrj[j].Time-dtMin)>-5.0)
				{
					nIndex = j+nReaded;
					bMinX = TRUE;
					break;
				}
				
			}
			if(!bMinX)
			{
				//对下次读取的记录数目进行重新确定
				nReaded += nReadStep;
				if(nReadStep+nReaded>=m_nRecord)
				{
					
					nReadStep = m_nRecord-nReaded;
				}
			}
			delete []pTempTrj;
			pTempTrj = NULL;
		}
		//如果找到了最小值以后，则从找到的位置开始寻找最大时间对应的位置
		if(bMinX)
		{
			m_fstream.seekg(m_TrajHdr.HdrSize+64*nIndex,ios::beg);
			nReaded = nIndex;
			nReadStep = 100000;
			if(m_nRecord<nReaded+nReadStep)
			{
				nReadStep = m_nRecord-nReaded;
				m_pTrj = new TrajPos[nReadStep];
			    m_fstream.read(( char*)m_pTrj,nReadStep*64);
				m_nRecord = nReadStep;
				return true;
			}
			else
			{
				while(!bFinded)
				{
					pTempTrj = new TrajPos[nReadStep];
					m_fstream.read(( char*)pTempTrj,nReadStep*64);
					for(int j=0;j<nReadStep;j++)
					{
						if(pTempTrj[j].Time>dtMax)
						{
							
							nMaxIndex = j+nReaded;
							bFinded = TRUE;
							break;
						}
						
					}
					if(!bFinded)
					{
						//对下次读取的记录数目进行重新确定
						nReaded += nReadStep;
						if(nReadStep+nReaded>=m_nRecord)
						{
							
							nReadStep = m_nRecord-nReaded;
						}
					}
					delete []pTempTrj;
					pTempTrj = NULL;

				}
				if(bFinded)
				{

					m_pTrj = new TrajPos[nMaxIndex-nIndex+1];
					m_fstream.seekg(m_TrajHdr.HdrSize+64*nIndex,ios::beg);
					m_nRecord = nMaxIndex-nIndex+1;
					m_fstream.read(( char*)m_pTrj,m_nRecord*64);
					return true;
				}
				else
				{

					return false;
				}
			}
		}
		else
		{

			return false;
		}

	}
	return true;
}


BOOL CTScanTrj::Open(string filename,string strLiDARFile,double dtMin,double dtMax)
{
	Close();
	bool bRet = TRUE;
	FILE* m_file = fopen(filename.c_str(),"r");
	if(!m_file)return false;
	char* pBuffer = new char[960];
	char* ptemp = pBuffer;
	fread(pBuffer,1,960,m_file);
	memcpy(m_TrajHdr.Recog,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.Version,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.HdrSize,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosCnt,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.PosSize,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.Desc,ptemp,79);
	ptemp += 79;
	memcpy(&m_TrajHdr.Quality,ptemp,1);
	ptemp += 1;
	memcpy(&m_TrajHdr.BegTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.EndTime,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.OrigNbr,ptemp,4);
	ptemp += 4;
	memcpy(&m_TrajHdr.Number,ptemp,4);
	ptemp += 4;
	memcpy(m_TrajHdr.VrtVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.VrtBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.VrtEnd,ptemp,8);
	ptemp += 8;
	memcpy(m_TrajHdr.FwdVideo,ptemp,400);
	ptemp += 400;
	memcpy(&m_TrajHdr.FwdBeg,ptemp,8);
	ptemp += 8;
	memcpy(&m_TrajHdr.FwdEnd,ptemp,8);
	ptemp += 8;
	fseek(m_file,0,SEEK_END);
	long lSize = ftell(m_file);
	fclose(m_file);
	m_nRecord = m_TrajHdr.PosCnt;

    m_nCurPos = 0;
	if(m_fstream.is_open())m_fstream.close();
	//m_fstream.open(filename.c_str(),ios::in|ios::out|ios::binary,filebuf::sh_read|filebuf::sh_write);
	m_fstream.open(filename.c_str(),ios::in|ios::out|ios::binary);
	m_bOpen = true;
	//
	m_fstream.seekg(m_TrajHdr.HdrSize,ios::beg);	
	m_bOpen = true;
	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	//读取一部分，判断里面是否包含所需要的记录
	int nReaded = 0;
	int nReadStep = 100000;
	if(m_TrajHdr.BegTime>dtMin||m_TrajHdr.EndTime<dtMax)
	{
		//文件范围里面的数据比所要求的范围小 则以失败返回
		m_fstream.close();
		m_bOpen = false;
		CString strTemp(filename.c_str());
		strTemp="存在时间范围和航迹文件不一致的航带"+strLiDARFile+"，请查对!";
		cout << strTemp << endl;
		return false;
	}

	if(nReadStep>=m_nRecord)
	{
		//记录数比较小的简单情形
		m_pTrj = new TrajPos[m_nRecord];
		m_fstream.read(( char*)m_pTrj,m_nRecord*64);
	}
	else
	{
		//记录数比较多的复杂情形
		TrajPos* pTempTrj = NULL;
		TrajPos* pTrjRecord = NULL;
		BOOL bFinded = FALSE;
		BOOL bMinX = FALSE;
		int nIndex = 0;
		int nMaxIndex = 0;
		while(!bMinX&&nReaded<m_nRecord)
		{
			pTempTrj = new TrajPos[nReadStep];
			m_fstream.read(( char*)pTempTrj,nReadStep*64);
			for(int j=0;j<nReadStep;j++)
			{
				if((pTempTrj[j].Time-dtMin)>-5.0)
				{
					nIndex = j+nReaded;
					bMinX = TRUE;
					break;
				}
				
			}
			if(!bMinX)
			{
				//对下次读取的记录数目进行重新确定
				nReaded += nReadStep;
				if(nReadStep+nReaded>=m_nRecord)
				{
					
					nReadStep = m_nRecord-nReaded;
				}
			}
			delete []pTempTrj;
			pTempTrj = NULL;
		}
		//如果找到了最小值以后，则从找到的位置开始寻找最大时间对应的位置
		if(bMinX)
		{
			m_fstream.seekg(m_TrajHdr.HdrSize+64*nIndex,ios::beg);
			nReaded = nIndex;
			nReadStep = 100000;
			if(m_nRecord<nReaded+nReadStep)
			{
				nReadStep = m_nRecord-nReaded;
				m_pTrj = new TrajPos[nReadStep];
			    m_fstream.read(( char*)m_pTrj,nReadStep*64);
				m_nRecord = nReadStep;
				return true;
			}
			else
			{
				while(!bFinded)
				{
					pTempTrj = new TrajPos[nReadStep];
					m_fstream.read(( char*)pTempTrj,nReadStep*64);
					for(int j=0;j<nReadStep;j++)
					{
						if(pTempTrj[j].Time>dtMax)
						{
							
							nMaxIndex = j+nReaded;
							bFinded = TRUE;
							break;
						}
						
					}
					if(!bFinded)
					{
						//对下次读取的记录数目进行重新确定
						nReaded += nReadStep;
						if(nReadStep+nReaded>=m_nRecord)
						{
							
							nReadStep = m_nRecord-nReaded;
						}
					}
					delete []pTempTrj;
					pTempTrj = NULL;

				}
				if(bFinded)
				{

					m_pTrj = new TrajPos[nMaxIndex-nIndex+1];
					m_fstream.seekg(m_TrajHdr.HdrSize+64*nIndex,ios::beg);
					m_nRecord = nMaxIndex-nIndex+1;
					m_fstream.read(( char*)m_pTrj,m_nRecord*64);
					return true;
				}
				else
				{

					return false;
				}
			}
		}
		else
		{

			return false;
		}

	}
	return true;
}
BOOL CTScanTrj::GetRecord(TrajPos& trj,double t)
{

	if(!m_pTrj)
	{
		return FALSE;
	}
	if(m_pTrj[0].Time>t||m_pTrj[m_nRecord-1].Time<t)return FALSE;
	BOOL bRet = FALSE;
	int i = m_nCurRecord;
	if(m_pTrj[i+1].Time>=t&&m_pTrj[i].Time<=t)
	{
		bRet = TRUE;
	}
	else
	{
		for(i=0;i<m_nRecord-1;i++)
		{
			if(m_pTrj[i+1].Time>=t&&m_pTrj[i].Time<=t)
			{
				bRet = TRUE;
				m_nCurRecord=i;
				break;
			}
		}
	}
	double dTRange = 0.0;
	double dt = 0.0;
	if(bRet)
	{
		trj.Time = t;
		dTRange = m_pTrj[i+1].Time - m_pTrj[i].Time;
		dt = (t-m_pTrj[i].Time)/dTRange;
		trj.Mark = m_pTrj[i].Mark;
		trj.Pitch = (1.0-dt)*m_pTrj[i].Pitch+dt*m_pTrj[i+1].Pitch;
		trj.Roll = (1.0-dt)*m_pTrj[i].Roll+dt*m_pTrj[i+1].Roll;
		trj.Head = (1.0-dt)*m_pTrj[i].Head+dt*m_pTrj[i+1].Head;
		trj.Quality = m_pTrj[i].Quality;
		trj.Xyz.x = (1.0-dt)*m_pTrj[i].Xyz.x+dt*m_pTrj[i+1].Xyz.x;
		trj.Xyz.y = (1.0-dt)*m_pTrj[i].Xyz.y+dt*m_pTrj[i+1].Xyz.y;
		trj.Xyz.z = (1.0-dt)*m_pTrj[i].Xyz.z+dt*m_pTrj[i+1].Xyz.z;
	}
	if(bRet)return TRUE;
	return FALSE;
}

long CTScanTrj::GetRecordNum()
{
	if(m_bOpen)
	return m_nRecord;
	return 0;
}



bool CTScanTrj::GetNext(TrajPos& trj)
{
	if(!m_bOpen)return false;
	if(m_nCurPos>=m_nRecord) return false;
	m_fstream.read(( char*)&trj,64);
	m_nCurPos++;
	return true;
}



void CTScanTrj::Close()
{
	if(m_fstream.is_open())m_fstream.close();
	m_nRecord = 0;
	m_bOpen = false;
	if(m_pTrj)delete []m_pTrj;
	m_pTrj = NULL;
	return;
}



bool CTScanTrj::GetRecord(int nPos,int nNum,TrajPos* pTrj)
{
	if(nPos<0) return false;
	if(nPos>=m_nRecord)return false;
	if(nNum<1) return false;
	if((nPos+nNum)>m_nRecord) return false;
	if(nPos!=m_nCurPos)
	m_fstream.seekg(960+nPos*64,ios::beg);
	m_fstream.read(( char*)pTrj,nNum*64);
	m_nCurPos = nPos+nNum;
	return true;
}



bool CTScanTrj::GetNext(OrientInfo& orient)
{
	TrajPos trj;
	bool bRet = GetNext(trj);
	if(!bRet)
	return false;
	orient.t = trj.Time;
	orient.lat = trj.Xyz.y;
	orient.lon = trj.Xyz.x;
	orient.alt = trj.Xyz.z;
	orient.r = trj.Roll*PI/180.0;
	orient.p = trj.Pitch*PI/180.0;
	orient.h = trj.Head*PI/180.0;
	return true;
}



bool CTScanTrj::GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient)
{
	TrajPos* pTrj = new TrajPos[nNum];
	bool bRet = GetRecord(nPos,nNum,pTrj);
	if(bRet==false)
	{
		delete []pTrj;
		pTrj = NULL;
		return false;
	}
	for(int i=0;i<nNum;i++)
	{
		pOrient[i].t = pTrj[i].Time;
		pOrient[i].lat = pTrj[i].Xyz.y;
		pOrient[i].lon = pTrj[i].Xyz.x;
		pOrient[i].alt = pTrj[i].Xyz.z;
		pOrient[i].r = pTrj[i].Roll*PI/180.0;
		pOrient[i].p = pTrj[i].Pitch*PI/180.0;
		pOrient[i].h = pTrj[i].Head*PI/180.0;
	}
	delete []pTrj;
	pTrj = NULL;
	return true;
}


TrajPos* CTScanTrj::GetRecord(double t)
{
	if(!m_pTrj)return NULL;
	if(m_pTrj[0].Time>t||m_pTrj[m_nRecord-1].Time<t)return NULL;
	TrajPos* pTrj = new TrajPos;
	BOOL bRet = FALSE;
	int i = m_nCurRecord;
	if(m_pTrj[i+1].Time>=t&&m_pTrj[i].Time<=t)
	{
		bRet = TRUE;
	}
	else
	for(i=0;i<m_nRecord-1;i++)
	{
		if(m_pTrj[i+1].Time>=t&&m_pTrj[i].Time<=t)
		{
			bRet = TRUE;
			m_nCurRecord=i;
			break;
		}
	}
	double dTRange = 0.0;
	double dt = 0.0;
	if(bRet)
	{
		pTrj->Time = t;
		dTRange = m_pTrj[i+1].Time - m_pTrj[i].Time;
		dt = (t-m_pTrj[i].Time)/dTRange;
		pTrj->Mark = m_pTrj[i].Mark;
		pTrj->Pitch = (1.0-dt)*m_pTrj[i].Pitch+dt*m_pTrj[i+1].Pitch;
		pTrj->Roll = (1.0-dt)*m_pTrj[i].Roll+dt*m_pTrj[i+1].Roll;
		pTrj->Head = (1.0-dt)*m_pTrj[i].Head+dt*m_pTrj[i+1].Head;
		pTrj->Quality = m_pTrj[i].Quality;
		pTrj->Xyz.x = (1.0-dt)*m_pTrj[i].Xyz.x+dt*m_pTrj[i+1].Xyz.x;
		pTrj->Xyz.y = (1.0-dt)*m_pTrj[i].Xyz.y+dt*m_pTrj[i+1].Xyz.y;
		pTrj->Xyz.z = (1.0-dt)*m_pTrj[i].Xyz.z+dt*m_pTrj[i+1].Xyz.z;
	}
	if(bRet)return pTrj;
	delete pTrj;
	pTrj = NULL;
	return NULL;
}



/***************************************************************************
 *	快速获取姿态信息，在内部建立索引
 *************************************************************************/

TrajPos* CTScanTrj::GetFastRecord(double t)

{
	if(m_nCurRecord==0)m_nCurRecord += 1;
	if(m_nCurRecord==m_nRecord-1)m_nCurRecord = m_nCurRecord-1;
	if(m_nCurRecord<0||m_nCurRecord>=m_nRecord||!m_pTrj)
	{
		return NULL;
	}
	return NULL;
}


/**************************************************************************************
*
*	
*
*************************************************************************************
BOOL SBET2Trj(CString strSBET,CString strTrj,double dLonCenter)

{
	Trajectory rec;
	FILE* flSBET = fopen(strSBET.GetBuffer(strSBET.GetLength()),"rb");//读入的out文件
	FILE* flTrj = fopen(strTrj.GetBuffer(strTrj.GetLength()),"wb");//写入的trj文件
	if(!flSBET)return FALSE;
	if(!flTrj) return FALSE;
	fseek(flSBET,0,SEEK_END);
	long filelen = ftell(flSBET);
	fseek(flSBET,0,SEEK_SET);

	int temp = 0;
	int nReadSize = 0;
	int DataSize = sizeof(Trajectory);
	//write trj header
	TrajHdr hdr;
	memcpy(hdr.Recog,"TSCANTRJ",8);
	hdr.Version = 20010715;
	hdr.HdrSize = sizeof(TrajHdr);
	hdr.PosCnt = filelen/sizeof(Trajectory);
	hdr.PosSize = sizeof(TrajPos);
	memset(hdr.Desc,'\0',79);
	hdr.Quality = 3;
	fseek(flSBET,0,SEEK_SET);
	fread(&rec,sizeof(Trajectory),1,flSBET);
	hdr.BegTime = rec.t;
	fseek(flSBET,(-1)*(int)(sizeof(Trajectory)),SEEK_END);
	fread(&rec,sizeof(Trajectory),1,flSBET);
	hdr.EndTime = rec.t;
	hdr.OrigNbr = 1;
	hdr.Number = 1;
	memset(hdr.VrtVideo,'\0',400);
	memset(hdr.FwdVideo,'\0',400);
	fwrite(&hdr,sizeof(TrajHdr),1,flTrj);
	//开始转化
	TrajPos trjPos;
	fseek(flSBET,0,SEEK_SET);
	ALSMapPrj prj;
	double tLat=0.0,tLon=0.0;
	double dCorrHeading=0.0;
	double e2=0.0067394967565869;
	//ALSGet
	LasProgress* laspro = GetLasProgressDlg("生成Trj文件",NULL);
	laspro->Start("生成Trj文件",hdr.PosCnt/3000+1);
	int nStep = 1;
	//进度条显示过快，加大进度条步长
	while (!feof(flSBET))
	{
		nReadSize = fread(&rec,DataSize,1,flSBET);
		tLat = rec.lat;
		tLon = rec.lon-dLonCenter;
		dCorrHeading = tLon*sin(tLat)+tLon*tLon*tLon*sin(tLat)*cos(tLat)*cos(tLat)*(1.0+3*e2*cos(tLat)*cos(tLat))/3.0;
		trjPos.Head = (rec.head-dCorrHeading)*DPI;
		trjPos.Roll = rec.roll*DPI;
		trjPos.Pitch = rec.pitch*DPI;
		prj.Convert_Geodetic_To_Transverse_Mercator(rec.lat,rec.lon,&trjPos.Xyz.x,&trjPos.Xyz.y);
 		trjPos.Time = rec.t;
 		trjPos.Xyz.z = rec.alt;
 
 		trjPos.Quality = 3;
 		trjPos.Mark = 1;
		fwrite(&trjPos,sizeof(TrajPos),1,flTrj);
		if(!(nStep%3000))
		{

			if(!laspro->SetInformation(nStep/3000,"正在将原始POS信息转换成Trj文件"))
			{

				fclose(flSBET);
				fclose(flTrj);
				laspro->End();
				return FALSE;
			}
		}
		nStep++;

	}
	laspro->SetInformation(hdr.PosCnt/3000+1,"已生成Trj文件");
	laspro->End();
	fclose(flSBET);
	fclose(flTrj);	
	return TRUE;
}



*/





