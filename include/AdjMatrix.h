/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-14 17:19:44
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-16 19:59:29
 * @FilePath: \cpp\src\buildAdjMatrix.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Vector2d POINT2d;
typedef Eigen::MatrixXd POINT2d_LIST;
typedef std::vector<POINT2d> LINE;
typedef std::vector<LINE> LINES;
typedef Eigen::MatrixXd ADJMAT;

typedef Eigen::VectorXi SHORTESTPATH_INDEX;

namespace AdjMatrix
{
    bool isBetween(const POINT2d& a, const POINT2d& b, const POINT2d& c);
    bool isPointInsideSegment(const POINT2d& point, const LINE& segment);
    bool isPointInsideAnySegment(const POINT2d& point, const LINES& lines, const POINT2d& intersectionPoint);
    Eigen::MatrixXd calculateDistances(const POINT2d& point, const POINT2d_LIST& points_list, const LINES& lines);
    ADJMAT buildAdjacencyMatrix(const POINT2d& start_point, const POINT2d_LIST& intersectionPoints, const POINT2d& end_point, const LINES& lines);
    SHORTESTPATH_INDEX shortestPath(const ADJMAT& graph);
};
