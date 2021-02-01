#pragma once
#include"alscloud.h"
class Point2D
{
public:
	Point2D(){}
	Point2D(int ix, int iy)
	{
		this->x = ix;
		this->y = iy;
		planeID = 0;
	}
	Point2D(int ix, int iy,int index)
	{
		this->x = ix;
		this->y = iy;
		planeID = index;
	}
	~Point2D(){}

	Point2D operator+(const Point2D& a) const
	{
		return Point2D(x + a.x, y + a.y,planeID);
	}
	int x, y;
	int planeID;
};


class RegionGrow
{
public:
	RegionGrow(flat_grid** data,int w, int h)
		:voxel(data),width(w),height(h),planeNum(0)
	{}
	~RegionGrow(){}

	//void SetInputData(flat_grid** voxel, int width, int height);
	bool SeekSeed();
	bool RegionGrow2D();
	int planeNum;
	map<int, vector<Point2D>> voxel_index;	
private:
	int width, height;
	Point2D seedPoint;
	flat_grid** voxel;
};
