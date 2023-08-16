/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-14 16:49:29
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-16 19:55:34
 * @FilePath: \cpp\test\testBuildAdjanceMatrix.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <AdjMatrix.h>

void printShortestPath(const std::vector<Eigen::Vector2d>& path) {
    std::cout << "The shortest path: ";
    for (const auto& point : path) {
        std::cout << "(" << point.x() << ", " << point.y() << ") ";
    }
    std::cout << std::endl;
}

void printValues(const Eigen::Vector2d& startPoint, const Eigen::MatrixXd& intersectionPoint, const Eigen::Vector2d& endPoint, const std::vector<std::vector<Eigen::Vector2d>>& lines) {
    std::cout << "Start Point: " << startPoint.transpose() << std::endl;

    std::cout << "Intersection Points: " << std::endl;
    for (int i = 0; i < intersectionPoint.cols(); ++i) {
        std::cout << intersectionPoint.col(i).transpose() << std::endl;
    }

    std::cout << "End Point: " << endPoint.transpose() << std::endl;

    std::cout << "Lines: " << std::endl;
    for (const auto& line : lines) {
        for (const auto& point : line) {
            std::cout << point.transpose() << " ";
        }
        std::cout << std::endl;
    }
}

int main()
{
    Eigen::Vector2d start_point(2);
    start_point << 1.5, 1.5;

    Eigen::MatrixXd intersectionPoints(4, 2);
    intersectionPoints << 2.6, 2.6,
                          3.6, 3.6,
                          3.2, 2.2,
                          4.0, 3.0;

    Eigen::Vector2d end_point(2);
    end_point << 5.25, 4.25;

    std::vector<std::vector<Eigen::Vector2d>> lines = {
        {Eigen::Vector2d{1, 1}, Eigen::Vector2d(4, 4)},
        {Eigen::Vector2d(2, 3), Eigen::Vector2d(5, 1)},
        {Eigen::Vector2d(3, 2), Eigen::Vector2d(6, 5)},
        {Eigen::Vector2d(4, 3), Eigen::Vector2d(2, 6)}
    };

    printValues(start_point, intersectionPoints, end_point, lines); 
    

    Eigen::MatrixXd adjMatrix = AdjMatrix::buildAdjacencyMatrix(start_point, intersectionPoints, end_point, lines);

    // 输出邻接矩阵
    std::cout << adjMatrix << std::endl;
    
    Eigen::VectorXi pathIndex = AdjMatrix::shortestPath(adjMatrix);

    // 打印最短路径序列
    std::cout << "shortest path index: " << pathIndex.transpose() << std::endl;

    std::vector<Eigen::Vector2d> path;
    int pathLen = intersectionPoints.rows() + 2;
    for (int i = 0; i < pathIndex.size(); ++i) {

        if(pathIndex[i] == 1) path.push_back(start_point);
        else if(pathIndex[i] == pathLen) path.push_back(end_point);
        else path.push_back(intersectionPoints.row(pathIndex[i]-2));
    }



    printShortestPath(path);
    return 0;
}