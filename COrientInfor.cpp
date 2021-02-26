#include <atlstr.h>
#include "COrientInfor.h"
#include "CTrajectory.h"
COrientInfor::COrientInfor()
{
	m_pTrj = NULL;
}
COrientInfor::COrientInfor(string filename)
{
	m_pTrj = NULL;
	CString strFile(filename.c_str());
	CString strZ = strFile.Right(3);
	strZ.MakeUpper();
	if(strZ.Compare(LPCTSTR("TRJ")))
	{
		m_pTrj = new CTScanTrj;
		
	}
	if(strZ.Compare(LPCTSTR("OUT")))
	{
		m_pTrj = new CPosPacTrj;
	}
	if(m_pTrj)
	m_pTrj->Open(filename);
}
COrientInfor::~COrientInfor()
{
	Close();
}
bool COrientInfor::Open(string filename)
{
	if(m_pTrj)delete m_pTrj;
	m_pTrj = NULL;
	CString strFile(filename.c_str());
	CString strZ = strFile.Right(3);
	strZ.MakeUpper();
	if(strZ.Compare(LPCTSTR("TRJ")))
	{
		m_pTrj = new CTScanTrj;
		
	}
	if(strZ.Compare(LPCTSTR("OUT")))
	{
		m_pTrj = new CPosPacTrj;
	}
	if(m_pTrj)
	return m_pTrj->Open(filename);
	return false;
}
bool COrientInfor::GetNext(OrientInfo& orient)
{
	if(!m_pTrj)return false;
	return m_pTrj->GetNext(orient);
}
long COrientInfor::GetRecordNum()
{
	if(!m_pTrj)return 0;
	return m_pTrj->GetRecordNum();
}
bool COrientInfor::GetOrientInfor(int nPos,int nNum,OrientInfo* pOrient)
{
	if(!m_pTrj)return false;
	return m_pTrj->GetOrientInfor(nPos,nNum,pOrient);
}
void COrientInfor::Close()
{
	if(!m_pTrj)return;
	m_pTrj->Close();
	delete m_pTrj;
	m_pTrj = NULL;
}