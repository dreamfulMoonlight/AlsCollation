#include "alscloud.h"
#include"RegionGrow.h"

#ifndef Agrid_distance
#define Agrid_distance  1.1
#endif



alscloud::~alscloud()
{
	
}

alscloud::alscloud(string s1)
	:als_cloud(new pcl::PointCloud<pcl::PointXYZI>)
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
	//��las�ļ�
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
	cout << "��ȡALS��������:" << i_1 << endl;
	//return als_cloud;
}

void alscloud::visual() {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(als_path));

	pcl::visualization::PointCloudColorHandlerGenericField<ptype> fildColor(als_cloud, "z"); // ����z�ֶν�����Ⱦ

	viewer->addPointCloud<ptype>(als_cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // ���õ��ƴ�С

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void alscloud::radiusFilter()
{
	ptrtype als_filter(new pcl::PointCloud<pcl::PointXYZI>);
	//�뾶�˲�
	pcl::RadiusOutlierRemoval<ptype> outrem;  //�����˲���
	outrem.setInputCloud(als_cloud);    //�����������
	outrem.setRadiusSearch(3);     //���ð뾶Ϊ0.5�ķ�Χ�����ٽ���
	outrem.setMinNeighborsInRadius(10); //���ò�ѯ�������㼯��С��10��ɾ��
	outrem.setNegative(false);
	// apply filter
	outrem.filter(*als_filter);     //ִ�������˲�   �ڰ뾶Ϊ0.8 �ڴ˰뾶�ڱ���Ҫ�������ھӵ㣬�˵�Żᱣ��
	als_cloud = als_filter;
	std::cout << "Cloud after filtering" << endl;
	std::cout << als_cloud->size() << endl;
}

void alscloud::planeExtract()
{
	//��ȡ������ֵ
	ptype min, max;
	pcl::getMinMax3D(*als_cloud, min, max);
	
	//���������ڸ���XYZ��������
	cout << "X�������ֵ��" << max.x << endl;
	cout << "X������Сֵ��" << min.x << endl;
	int width = int((max.x - min.x) / Agrid_distance) + 1;

	cout << "Y�������ֵ��" << max.y << endl;
	cout << "Y������Сֵ��" << min.y << endl;
	int height = int((max.y - min.y) / Agrid_distance)+1;

	cout << "�������߲�:" << max.z - min.z << endl;
	//������άƽ�����
	flat_grid** voxel_2 = new flat_grid * [width];
	for (int i = 0; i < width; ++i)
		voxel_2[i] = new flat_grid[height];
	int row_als, col_als;
	for (size_t i = 0; i < als_cloud->points.size(); i++)
	{
		row_als = int((als_cloud->points[i].x - min.x) / Agrid_distance);
		col_als = int((als_cloud->points[i].y - min.y) / Agrid_distance);
		voxel_2[row_als][col_als].indexID.push_back(i);
		if (voxel_2[row_als][col_als].grayScale < 1)
		{
			voxel_2[row_als][col_als].grayScale++; 
		}
	}
	cout << "����������" << width * height << endl;
	int count_grid = 0;
	pcl::PointIndices::Ptr pointIndices_als(new pcl::PointIndices());
	float roofHeight = 0;
	cout << "�����ݶ��߳�:" << endl;
	cin >> roofHeight;
	//��ȡ�ݶ����ص���
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			if (voxel_2[i][j].grayScale == 1)
			{
				count_grid++;
				pcl::PointCloud<pcl::PointXYZI>::Ptr voxelPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);   //�����������Ƽ�
				voxelPointCloudPtr->width = voxel_2[i][j].indexID.size();
				voxelPointCloudPtr->height = 1;
				voxelPointCloudPtr->is_dense = false;
				voxelPointCloudPtr->resize(voxelPointCloudPtr->width * voxelPointCloudPtr->height);
				for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //��ȡ������������
				{

					voxelPointCloudPtr->points[k].x = als_cloud->points[voxel_2[i][j].indexID[k]].x;
					voxelPointCloudPtr->points[k].y = als_cloud->points[voxel_2[i][j].indexID[k]].y;
					voxelPointCloudPtr->points[k].z = als_cloud->points[voxel_2[i][j].indexID[k]].z;
				}
				ptype voxel_min;
				ptype voxel_max;
				pcl::getMinMax3D(*voxelPointCloudPtr, voxel_min, voxel_max);
				if (voxel_max.z - min.z >= roofHeight)
				{
					voxel_2[i][j].candidate = 1;
					for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //��ȡ������������
					{
						//if (voxelPointCloudPtr->points[k].z - min.z < roofHeight)voxel_2[i][j].indexID.erase(voxel_2[i][j].indexID.begin() + k);
						if (voxelPointCloudPtr->points[k].z - min.z >= roofHeight) pointIndices_als->indices.push_back(voxel_2[i][j].indexID[k]);
					}
				}
				
			}
		}
	}
	cout << "�����������" << count_grid << endl;
	/*
	pcl::PointCloud<pcl::PointXYZI>::Ptr Acloud_flitered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ExtractIndices<pcl::PointXYZI> Aextract;
	// Extract the inliers
	Aextract.setInputCloud(als_cloud);
	Aextract.setIndices(pointIndices_als);
	Aextract.setNegative(false);//�����Ϊtrue,������ȡָ��index֮��ĵ���
	Aextract.filter(*Acloud_flitered);
	als_cloud = Acloud_flitered;
	cout << "�����������:" << Acloud_flitered->size() << endl;
	*/
	RegionGrow s1(voxel_2,width,height);
	if (s1.RegionGrow2D())
	{
		cout << "���ƽ��������" << s1.planeNum << endl;
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
				if (als_cloud->points[voxel_2[v_x][v_y].indexID[k]].z > max_h) max_h = als_cloud->points[voxel_2[v_x][v_y].indexID[k]].z;
				if (als_cloud->points[voxel_2[v_x][v_y].indexID[k]].z < min_h) min_h = als_cloud->points[voxel_2[v_x][v_y].indexID[k]].z;
			}
			if (max_h - min_h > 5)
			{
				for (size_t k = 0; k < voxel_2[v_x][v_y].indexID.size(); k++)
				{
					ptype p1 = als_cloud->points[voxel_2[v_x][v_y].indexID[k]];
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
	cout << "��ȡ�ݶ������������" << count << endl;
	als_cloud = voxelPointCloudPtr;
	visual();


	for (int i = 0; i < width; ++i)
		delete[] voxel_2[i];
	delete[] voxel_2;


}

AlsManager::AlsManager(alscloud s1, alscloud s2)
	:als_1 (s1), als_2(s2)
{

}

void AlsManager::cloudvisual(const char* name)
{
	ptrtype src(als_1.getcloud()), tgt(als_2.getcloud());
	//�����Ӵ����󲢸�����������һ�����ơ�3D Viewer������������Ϊboost::shared_ptr���ܹ���ָ�룬�������Ա�ָ֤���ڳ�����ȫ��ʹ�ã����������ڴ����
	pcl::visualization::PCLVisualizer viewer(name);
	//�����Ӵ��ı���ɫ��������������RGB����ɫ������������Ϊ��ɫ
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<ptype> target_color(tgt, 0, 255, 0);

	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//��������ӵ��Ӵ������У�����һ��Ψһ���ַ�����ΪID �ţ����ô��ַ�����֤��������Ա��Ҳ�ܱ�־���øõ��ƣ���ε���addPointCloud����ʵ�ֶ�����Ƶ���ӣ�ÿ����һ�ξͻᴴ��һ���µ�ID�ţ���������һ���Ѿ���ʾ�ĵ��ƣ��ȵ���removePointCloud���������ṩ��Ҫ���µĵ���ID �ţ�Ҳ��ʹ��updatePointCloud
	viewer.addPointCloud<ptype>(tgt, target_color, "target cloud", 1);
	//���ڸı���ʾ���Ƶĳߴ磬�������ø÷������Ƶ������Ӵ��е���ʾ����,1������ʾ���ƴ�С
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

	pcl::visualization::PointCloudColorHandlerCustom<ptype> source_color(src, 255, 0, 0);
	//��������ӵ��Ӵ������У�����һ��Ψһ���ַ�����ΪID �ţ����ô��ַ�����֤��������Ա��Ҳ�ܱ�־���øõ��ƣ���ε���addPointCloud����ʵ�ֶ�����Ƶ���ӣ�ÿ����һ�ξͻᴴ��һ���µ�ID�ţ���������һ���Ѿ���ʾ�ĵ��ƣ��ȵ���removePointCloud���������ṩ��Ҫ���µĵ���ID �ţ�Ҳ��ʹ��updatePointCloud
	viewer.addPointCloud<ptype>(src, source_color, "source cloud", 2);
	//���ڸı���ʾ���Ƶĳߴ磬�������ø÷������Ƶ������Ӵ��е���ʾ����,1������ʾ���ƴ�С
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

void AlsManager::Hcollation()
{
	//��ȡ������ֵ
}