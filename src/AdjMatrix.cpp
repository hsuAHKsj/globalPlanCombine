/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-14 17:19:44
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-16 19:59:01
 * @FilePath: \cpp\src\buildAdjMatrix.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <AdjMatrix.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <queue>
#include <iostream>

bool AdjMatrix::isBetween(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    double epsilon = 1e-9;
    double crossproduct = (c(1) - a(1)) * (b(0) - a(0)) - (c(0) - a(0)) * (b(1) - a(1));

    if (std::abs(crossproduct) > epsilon)
    {
        return false;
    }

    double dotproduct = (c(0) - a(0)) * (b(0) - a(0)) + (c(1) - a(1)) * (b(1) - a(1));
    if (dotproduct < 0)
    {
        return false;
    }

    double squaredlengthba = (b(0) - a(0)) * (b(0) - a(0)) + (b(1) - a(1)) * (b(1) - a(1));
    if (dotproduct > squaredlengthba)
    {
        return false;
    }

    return true;
}

bool AdjMatrix::isPointInsideSegment(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& segment)
{
    double x = point(0);
    double y = point(1);
    double x1 = segment[0](0);
    double y1 = segment[0](1);
    double x2 = segment[1](0);
    double y2 = segment[1](1);

    bool isInside = (x >= std::min(x1, x2) && x <= std::max(x1, x2) &&
                     y >= std::min(y1, y2) && y <= std::max(y1, y2));

    return isInside;
}

bool AdjMatrix::isPointInsideAnySegment(const Eigen::Vector2d& point, const std::vector<std::vector<Eigen::Vector2d>>& lines, const Eigen::Vector2d& intersectionPoint)
{
    bool isInside = false;


    for (size_t i = 0; i < lines.size(); ++i)
    {
        const std::vector<Eigen::Vector2d>& line = lines[i];
        if (isPointInsideSegment(point, line) && isPointInsideSegment(intersectionPoint, line))
        {
            if (isBetween(line[0], line[1], point) &&
                isBetween(line[0], line[1], intersectionPoint))
            {
                isInside = true;
                break;
            }
        }
    }
    return isInside;
}



Eigen::MatrixXd AdjMatrix::calculateDistances(const Eigen::Vector2d& point, const Eigen::MatrixXd& points_list, const std::vector<std::vector<Eigen::Vector2d>>& lines)
{

    int num_points = points_list.rows();
    Eigen::MatrixXd distances(1, num_points);

    for (int i = 0; i < num_points; ++i)
    {
        if (isPointInsideAnySegment(point, lines, points_list.row(i)))
        {
            distances(0, i) = (point.transpose() - points_list.row(i)).norm();
        }
        else
        {
            distances(0, i) = std::numeric_limits<double>::infinity();
        }
    }
    return distances;
}

Eigen::MatrixXd AdjMatrix::buildAdjacencyMatrix(const Eigen::Vector2d& start_point, const Eigen::MatrixXd& intersectionPoints, const Eigen::Vector2d& end_point, const std::vector<std::vector<Eigen::Vector2d>>& lines)
{
    int numVertices = intersectionPoints.rows() + 2; // 交点数加上起点和终点
    int numPoints = intersectionPoints.rows();        // 通道交点的数目

    Eigen::MatrixXd adjMatrix = Eigen::MatrixXd::Zero(numVertices, numVertices); // 邻接矩阵初始化为0

    adjMatrix.block(0, 1, 1, numPoints) = calculateDistances(start_point, intersectionPoints, lines);
    adjMatrix.block(1, numVertices - 1, numPoints, 1) = calculateDistances(end_point, intersectionPoints, lines).transpose();

    // 对称性
    adjMatrix.block(1, 0, numPoints, 1) = adjMatrix.block(0, 1, 1, numPoints).transpose();
    adjMatrix.block(numVertices - 1, 1, 1, numPoints) = adjMatrix.block(1, numVertices - 1, numPoints, 1).transpose();


    adjMatrix.block(0, numVertices - 1, 1, 1) = calculateDistances(start_point, end_point.transpose(), lines);
    adjMatrix(numVertices - 1, 0) = adjMatrix(0, numVertices - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Eigen::Vector2d point = intersectionPoints.row(i);
        Eigen::MatrixXd m = intersectionPoints.block(i, 0, numPoints - i, 2);

        auto distances = calculateDistances(point, m, lines);

        adjMatrix.block(i + 1, i + 1, numPoints - i, 1) = distances.transpose();

        adjMatrix.block(i + 1, i + 1, 1, numPoints - i) = distances;
    }

    return adjMatrix;
}


// 邻接矩阵表示的图的最短路径函数
Eigen::VectorXi AdjMatrix::shortestPath(const Eigen::MatrixXd& graph) {
    int numVertices = graph.rows();

    // 存储从起始顶点到每个顶点的最短距离
    Eigen::VectorXd distances(numVertices);
    distances.setConstant(numVertices, std::numeric_limits<double>::infinity());

    // 存储最短路径的前驱顶点
    Eigen::VectorXi predecessors(numVertices);
    predecessors.setConstant(numVertices, -1);

    // 标记顶点是否已经被访问
    Eigen::VectorXi visited(numVertices);
    visited.setZero(numVertices);

    // 起始顶点为1，将其距离设为0
    distances(0) = 0.0;

    // 创建优先队列，按照距离从小到大排序
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push(std::make_pair(0.0, 0));

    while (!pq.empty()) {
        int currentVertex = pq.top().second;
        pq.pop();

        visited(currentVertex) = 1;

        // 更新相邻顶点的距离和前驱顶点
        for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
            double edgeWeight = graph(currentVertex, neighbor);

            if (std::isinf(edgeWeight) || visited(neighbor) == 1)
                continue;

            if (distances(currentVertex) != std::numeric_limits<double>::infinity() &&
                distances(currentVertex) + edgeWeight < distances(neighbor)) {
                distances(neighbor) = distances(currentVertex) + edgeWeight;
                predecessors(neighbor) = currentVertex;
                pq.push(std::make_pair(distances(neighbor), neighbor));
            }
        }
    }

    // 构造最短路径序列
    Eigen::VectorXi shortestPathSequence;
    int currentVertex = numVertices - 1;
    while (currentVertex != -1) {
        shortestPathSequence.conservativeResize(shortestPathSequence.size() + 1);
        shortestPathSequence(shortestPathSequence.size() - 1) = currentVertex + 1;
        currentVertex = predecessors(currentVertex);
    }

    return shortestPathSequence.reverse();
}