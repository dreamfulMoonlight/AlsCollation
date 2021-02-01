#include "alscloud.h"
#include"RegionGrow.h"

#ifndef Agrid_distance
#define Agrid_distance  1.1
#define roofHeight 25
#endif



alscloud::~alscloud()
{
	
}

alscloud::alscloud(string s1)
	:als_cloud(new pcl::PointCloud<pcl::PointXYZI>)
	,roof_cloud(new pcl::PointCloud<pcl::PointXYZI>)
	,road_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
	string in_als = s1 + ".las";
	als_path = in_als.c_str();
	readlas();
}

static std::string convertFilePath(const std::string& file)
{
	int i = 0;
	std::string s(file);
	for (i = 0; i < s.size(); ++i)
	{
		if (s[i] == '/')
			s[i] = '\\';
	}
	return s;
}

void alscloud::readlas()
{
	std::string lasFile_1 = convertFilePath(als_path);
	//打开las文件
	LASreadOpener lasreadopener_1;
	lasreadopener_1.set_file_name(lasFile_1.c_str());
	LASreader* lasreader_A = lasreadopener_1.open();
	size_t count_1 = lasreader_A->header.number_of_point_records;
	long long i_1 = 0;
	while (lasreader_A->read_point()) //&&lasreader_A->point.get_classification()==2)
	{
		int a = lasreader_A->point.get_classification();
		if (true)//a == 2)
		{
			pcl::PointXYZI als_point;
			als_point.x = lasreader_A->point.get_x();
			als_point.y = lasreader_A->point.get_y();
			als_point.z = lasreader_A->point.get_z();
			als_point.intensity = lasreader_A->point.get_intensity();
			//als_cloud->points[i_1].x = lasreader_A->point.get_x();
			//als_cloud->points[i_1].y = lasreader_A->point.get_y();
			//als_cloud->points[i_1].z = lasreader_A->point.get_z();
			//als_cloud->points[i_1].intensity = lasreader_A->point.get_intensity();
			als_cloud->points.push_back(als_point);
			++i_1;
		}
	}
	als_cloud->resize(i_1);
	als_cloud->width = i_1;
	als_cloud->height = 1;
	als_cloud->is_dense = false;
	//ptype min_als, max_als;
	//pcl::getMinMax3D(*als_cloud, min_als, max_als);
	cout << "读取ALS点云数量:" << i_1 << endl;
	//return als_cloud;
}

void alscloud::visual(ptrtype p1) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(als_path));

	pcl::visualization::PointCloudColorHandlerGenericField<ptype> fildColor(p1, "z"); // 按照z字段进行渲染

	viewer->addPointCloud<ptype>(p1, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void alscloud::radiusFilter()
{
	ptrtype als_filter(new pcl::PointCloud<pcl::PointXYZI>);
	//半径滤波
	pcl::RadiusOutlierRemoval<ptype> outrem;  //创建滤波器
	outrem.setInputCloud(als_cloud);    //设置输入点云
	outrem.setRadiusSearch(3);     //设置半径为0.5的范围内找临近点
	outrem.setMinNeighborsInRadius(10); //设置查询点的邻域点集数小于10的删除
	outrem.setNegative(false);
	// apply filter
	outrem.filter(*als_filter);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
	als_cloud = als_filter;
	std::cout << "Cloud after filtering" << endl;
	std::cout << als_cloud->size() << endl;
}
void alscloud::passthrough()
{
	//2.取得点云坐标极值
	ptype minPt, maxPt;
	pcl::getMinMax3D(*als_cloud, minPt, maxPt);

	//3.直通滤波
	ptrtype cloud_filter(new pcl::PointCloud<ptype>);
	pcl::PassThrough<ptype> pass;     //创建滤波器对象
	pass.setInputCloud(als_cloud);                //设置待滤波的点云
	pass.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
	pass.setFilterLimits(minPt.z+25,maxPt.z);    //设置滤波范围(从最高点向下12米去除)
	pass.setFilterLimitsNegative(false);      //保留
	pass.filter(*cloud_filter);               //滤波并存储
	als_cloud = cloud_filter;
	//visual();
}
void alscloud::roofExtract()
{
	roof_cloud = als_cloud;
	//提取点云最值
	ptype min, max;
	pcl::getMinMax3D(*roof_cloud, min, max);
	
	//计算区域内格网XYZ方向数量
	cout << "X方向最大值：" << max.x << endl;
	cout << "X方向最小值：" << min.x << endl;
	int width = int((max.x - min.x) / Agrid_distance) + 1;

	cout << "Y方向最大值：" << max.y << endl;
	cout << "Y方向最小值：" << min.y << endl;
	int height = int((max.y - min.y) / Agrid_distance)+1;

	cout << "区域最大高差:" << max.z - min.z << endl;
	//构建二维平面格网
	flat_grid** voxel_2 = new flat_grid * [width];
	for (int i = 0; i < width; ++i)
		voxel_2[i] = new flat_grid[height];
	int row_als, col_als;

	for (size_t i = 0; i < roof_cloud->points.size(); i++)
	{
		if ((roof_cloud->points[i].z - min.z > roofHeight))
		{
			row_als = int((roof_cloud->points[i].x - min.x) / Agrid_distance);
			col_als = int((roof_cloud->points[i].y - min.y) / Agrid_distance);
			voxel_2[row_als][col_als].indexID.push_back(i);
			if (voxel_2[row_als][col_als].grayScale < 1)
			{
				voxel_2[row_als][col_als].grayScale++;
			}
		}
	}
	cout << "格网数量：" << width * height << endl;
	int count_grid = 0;
	pcl::PointIndices::Ptr pointIndices_als(new pcl::PointIndices());
	
	//提取屋顶边沿点云
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			if (voxel_2[i][j].grayScale == 1)
			{
				count_grid++;
				pcl::PointCloud<pcl::PointXYZI>::Ptr voxelPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);   //构建格网点云集
				voxelPointCloudPtr->width = voxel_2[i][j].indexID.size();
				voxelPointCloudPtr->height = 1;
				voxelPointCloudPtr->is_dense = false;
				voxelPointCloudPtr->resize(voxelPointCloudPtr->width * voxelPointCloudPtr->height);
				for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //读取格网点云数据
				{

					voxelPointCloudPtr->points[k].x = roof_cloud->points[voxel_2[i][j].indexID[k]].x;
					voxelPointCloudPtr->points[k].y = roof_cloud->points[voxel_2[i][j].indexID[k]].y;
					voxelPointCloudPtr->points[k].z = roof_cloud->points[voxel_2[i][j].indexID[k]].z;
				}
				ptype voxel_min;
				ptype voxel_max;
				pcl::getMinMax3D(*voxelPointCloudPtr, voxel_min, voxel_max);
				if (voxel_max.z - min.z >= roofHeight)
				{
					voxel_2[i][j].candidate = 1;
				}
			}
		}
	}
	cout << "存在屋顶点云格网数：" << count_grid << endl;

	RegionGrow s1(voxel_2,width,height);
	if (s1.RegionGrow2D())
	{
		cout << "输出平面数量：" << s1.planeNum << endl;
	}

	ptrtype voxelPointCloudPtr(new pcl::PointCloud<ptype>);
	size_t count = 0;
	for (size_t i = 0; i < s1.voxel_index.size(); i++)
	{
		for (size_t j = 0; j < s1.voxel_index[i].size(); j++)
		{
			int v_x = s1.voxel_index[i][j].x, v_y = s1.voxel_index[i][j].y;
			double max_h=INT_MIN, min_h=INT_MAX;
			for (size_t k = 0; k < voxel_2[v_x][v_y].indexID.size(); k++)
			{
				if (roof_cloud->points[voxel_2[v_x][v_y].indexID[k]].z > max_h) max_h = roof_cloud->points[voxel_2[v_x][v_y].indexID[k]].z;
				if (roof_cloud->points[voxel_2[v_x][v_y].indexID[k]].z < min_h) min_h = roof_cloud->points[voxel_2[v_x][v_y].indexID[k]].z;
			}
			if (max_h - min_h < 1.0) //&& max_h - min_h > 1)  设置格网高程变化率
			{
				for (size_t k = 0; k < voxel_2[v_x][v_y].indexID.size(); k++)
				{
					ptype p1 = roof_cloud->points[voxel_2[v_x][v_y].indexID[k]];
					voxelPointCloudPtr->points.push_back(p1);
					count++;
				}
			}
		}
	}
	voxelPointCloudPtr->width = count;
	voxelPointCloudPtr->height = 1;
	voxelPointCloudPtr->is_dense = false;
	voxelPointCloudPtr->resize(voxelPointCloudPtr->width * 1);
	cout << "提取屋顶面点云数量：" << count << endl;
	roof_cloud = voxelPointCloudPtr;
	//visual();

	for (int i = 0; i < width; ++i)
		delete[] voxel_2[i];
	delete[] voxel_2;


}

void alscloud::roadExtract()
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<ptype> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	double distThreshold = 0.5;
	seg.setDistanceThreshold(distThreshold);
	seg.setInputCloud(als_cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<ptype> extract;
	extract.setInputCloud(als_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*road_cloud);
	visual(road_cloud);
}

AlsManager::AlsManager(alscloud s1, alscloud s2)
	:als_1 (s1), als_2(s2)
{

}

void AlsManager::cloudvisual(ptrtype src,ptrtype tgt,const char* name)
{
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	pcl::visualization::PCLVisualizer viewer(name);
	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<ptype> target_color(tgt, 0, 255, 0);
	/*
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	*/
	//将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个已经显示的点云，先调用removePointCloud（），并提供需要更新的点云ID 号，也可使用updatePointCloud
	viewer.addPointCloud<ptype>(tgt, target_color, "target cloud");
	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法,1设置显示点云大小
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

	pcl::visualization::PointCloudColorHandlerCustom<ptype> source_color(src, 255, 0, 0);
	//将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个已经显示的点云，先调用removePointCloud（），并提供需要更新的点云ID 号，也可使用updatePointCloud
	viewer.addPointCloud<ptype>(src, source_color, "source cloud");
	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法,1设置显示点云大小
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

void AlsManager::XYcollation()
{
	//提取点云最值
	vector<float> trans(3);
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	pcl::IterativeClosestPoint<ptype, ptype> icp;
	int max_iter = 30;
	icp.setMaximumIterations(max_iter);
	icp.setInputSource(als_1.roof_cloud);
	icp.setInputTarget(als_2.roof_cloud);
	icp.setEuclideanFitnessEpsilon(0.01);//前后两次迭代误差的差值
	//icp.setTransformationEpsilon(1e-10); //上次转换与当前转换的差值；
	icp.setMaxCorrespondenceDistance(10); //忽略在此距离之外的点，对配准影响较大
	icp.align(*als_1.roof_cloud);
	cloudvisual(als_1.roof_cloud,als_2.roof_cloud,"icp");
	Eigen::Matrix4f Mtransformation = icp.getFinalTransformation();
	trans[0] = Mtransformation(0, 3);
	trans[1] = Mtransformation(1, 3);
	trans[2] = Mtransformation(2, 3);
	cout << "matrix:\n" << icp.getFinalTransformation() << endl;
}

void AlsManager::Hcollation()
{
	//提取点云最值
	vector<float> trans(3);
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	pcl::IterativeClosestPoint<ptype, ptype> icp;
	int max_iter = 10;
	icp.setMaximumIterations(max_iter);
	icp.setInputSource(als_1.road_cloud);
	icp.setInputTarget(als_2.road_cloud);
	icp.setEuclideanFitnessEpsilon(0.01);//前后两次迭代误差的差值
	//icp.setTransformationEpsilon(1e-10); //上次转换与当前转换的差值；
	icp.setMaxCorrespondenceDistance(10); //忽略在此距离之外的点，对配准影响较大
	icp.align(*als_1.road_cloud);
	cloudvisual(als_1.road_cloud, als_2.road_cloud,"icp");
	Eigen::Matrix4f Mtransformation = icp.getFinalTransformation();
	trans[0] = Mtransformation(0, 3);
	trans[1] = Mtransformation(1, 3);
	trans[2] = Mtransformation(2, 3);
	cout << "matrix:\n" << icp.getFinalTransformation() << endl;
}