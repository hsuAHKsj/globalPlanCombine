/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-15 23:16:11
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 23:16:15
 * @FilePath: \cpp\src\utilityFun.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <utilityFun.h>

Eigen::Vector2d projectPointOntoLine(const Eigen::Vector2d& point, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point) {
    // 计算向量
    Eigen::Vector2d vector = end_point - start_point;
    Eigen::Vector2d vector_norm = vector.normalized();

    // 计算投影
    Eigen::Vector2d projection = start_point + (point - start_point).dot(vector_norm) * vector_norm;

    return projection;
}

bool isPointInsideBoundary(const Eigen::Vector2d& point, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, double r) {
    // 计算点到线段的向量
    Eigen::Vector2d line_vector = end_point - start_point;
    Eigen::Vector2d point_vector = point - start_point;

    // 计算点到线段的投影长度
    double projection_length = point_vector.dot(line_vector) / line_vector.norm();

    // 计算点到线段的距离
    double distance;
    if (projection_length <= 0) {
        // 点到线段起点的距离
        distance = point_vector.norm();
    } else if (projection_length >= line_vector.norm()) {
        // 点到线段终点的距离
        distance = (point - end_point).norm();
    } else {
        // 点到线段的垂直距离
        Eigen::Vector2d projection_vector = (projection_length / line_vector.norm()) * line_vector;
        distance = (point_vector - projection_vector).norm();
    }

    // 判断距离是否小于 r
    bool inside = (distance <= r);
    return inside;
}
