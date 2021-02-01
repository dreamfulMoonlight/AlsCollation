
#include "RegionGrow.h"
#include <stack>

const Point2D PointShift2D[8] =
{
    Point2D(1, 0),
    Point2D(1, -1),
    Point2D(0, -1),
    Point2D(-1, -1),
    Point2D(-1, 0),
    Point2D(-1, 1),
    Point2D(0, 1),
    Point2D(1, 1)
};

bool RegionGrow::SeekSeed()
{
    for(size_t i=0;i<width;i++)
        for (size_t j = 0; j < height; j++)
        {
            if (voxel[i][j].candidate == 1)
            {
                Point2D tempt(i, j, ++planeNum);
                seedPoint=tempt ;
                voxel[i][j].candidate = 0;
                return true;
            }
        }
    return false;
}

bool RegionGrow::RegionGrow2D()
{
    if (voxel == nullptr) return false;
    std::stack<Point2D> pointStack;
    while (SeekSeed())   //搜索种子点
    {
        vector<Point2D> cabinet;
        int count = 0;
        pointStack.push(seedPoint);
        while (!pointStack.empty())
        {
            Point2D topPoint = pointStack.top();
            pointStack.pop();
            for (int i = 0; i < 8; i++)
            {
                Point2D p = topPoint + PointShift2D[i];
                if (p.x > 0 && p.y > 0)
                {
                    if (voxel[p.x][p.y].candidate == 1)
                    {
                        pointStack.push(p);
                        voxel[p.x][p.y].candidate = 0;
                        cabinet.push_back(p);
                        count++;
                    }
                }
            }
        }
        if (count < 10) planeNum--;  //若选定区域格网数量小于一定阈值，视为树木或者其他非建筑物对象
        else {
            voxel_index[planeNum] = cabinet;   //满足区域面积则输出
        }
    }
    //cout << "提取平面数量:" << planeNum << endl;

    
    return true;
}