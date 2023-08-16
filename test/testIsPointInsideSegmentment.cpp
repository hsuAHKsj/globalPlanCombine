/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-14 16:54:22
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 20:35:52
 * @FilePath: \cpp\test\testIsPointInsideSegmentment.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "AdjMatrix.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace std;

void testIsPointInsideAnySegment()
{
    Eigen::Vector2d point(5.24, 4.25);

    std::vector<std::vector<Eigen::Vector2d>> lines = {
        {Eigen::Vector2d{1, 1}, Eigen::Vector2d(4, 4)},
        {Eigen::Vector2d(2, 3), Eigen::Vector2d(5, 1)},
        {Eigen::Vector2d(3, 2), Eigen::Vector2d(6, 5)},
        {Eigen::Vector2d(4, 3), Eigen::Vector2d(2, 6)}
    };

    cout << Eigen::Vector2d{1, 1} << endl;

    Eigen::Vector2d intersectionPoint(3.2, 2.2);

    bool isInside = AdjMatrix::isPointInsideAnySegment(point, lines, intersectionPoint);

    if (isInside)
    {
        std::cout << "The point is inside the segment." << std::endl;
    }
    else
    {
        std::cout << "The point is not inside the segment." << std::endl;
    }
}

int main()
{
    testIsPointInsideAnySegment();

    return 0;
}
