
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
    while (SeekSeed())   //�������ӵ�
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
        if (count < 10) planeNum--;  //��ѡ�������������С��һ����ֵ����Ϊ��ľ���������ǽ��������
        else {
            voxel_index[planeNum] = cabinet;   //����������������
        }
    }
    //cout << "��ȡƽ������:" << planeNum << endl;

    
    return true;
}