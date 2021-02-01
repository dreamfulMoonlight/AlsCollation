// AlsCollation.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include"alscloud.h"
int main()
{
    //设置las文件路径
    cout << "设置源机载文件" << endl;
    string in_als1;
    cin >> in_als1;
    cout << "设置目标机载文件" << endl;
    string in_als2;
    cin >> in_als2;

    //读取las文件并进行半径滤波处理
    alscloud als_1(in_als1), als_2(in_als2);
    als_1.radiusFilter();
    //als_1.visual();  
    als_2.radiusFilter();
    //als_2.visual();
    
    //提取屋顶面
    als_1.roofExtract();
    als_2.roofExtract();
    //提取路面
    als_1.roadExtract();
    als_2.roadExtract();

    AlsManager opt1(als_1, als_2);
    //进行平面配准
    opt1.XYcollation();
    //进行高程配准
    opt1.Hcollation();
    std::cout << "Hello World!\n";
}


