#pragma once

#include <pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>//可视化头文件
#include "lasreader.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>  //直通滤波相关
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include"CTrajectory.h"

using namespace std;
typedef pcl::PointXYZI ptype;
typedef pcl::PointCloud<ptype>::Ptr ptrtype;



class alscloud
{
public:
	alscloud() {}
	alscloud(string s1);
	//alscloud(ptrtype p1):als_cloud(p1){}
	~alscloud();
	void readlas();
	void radiusFilter();
	void visual(ptrtype p1);
	void passthrough();
	ptrtype getcloud() { return als_cloud; }
	void roofExtract();
	void roadExtract();
	ptrtype road_cloud, roof_cloud;
private:
	const char* als_path;
	ptrtype als_cloud;
};

class AlsManager
{
public:
	AlsManager() {}
	AlsManager(alscloud s1, alscloud s2);
	~AlsManager() {}
	void centralize(ptrtype mls_vscloud, ptrtype als_vscloud);
	void WriteLas();
	void cloudvisual(ptrtype src,ptrtype tgt, const char* name);
	void Hcollation();
	void XYcollation();
private:
	alscloud als_1, als_2;
	CTScanTrj trj;
};

typedef struct flat_grid {
	int grayScale = 0;
	float maxdif_height = 0;
	float max_height = 0;
	int candidate = 0;
	vector<int> indexID;
}flat_grid;