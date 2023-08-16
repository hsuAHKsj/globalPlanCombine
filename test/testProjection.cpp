/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-13 19:23:07
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-13 19:25:19
 * @FilePath: \cpp\test\testProjection.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEs
 */
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

Vector2d projectPointOntoLine(const Vector2d& point, const Vector2d& start_point, const Vector2d& end_point) {
    // 计算向量
    Vector2d vector = end_point - start_point;
    Vector2d vector_norm = vector.normalized();

    // 计算投影
    Vector2d projection = start_point + ((point - start_point).dot(vector_norm)) * vector_norm;

    return projection;
}

int main() {
    Vector2d point(2, 1);
    Vector2d start_point(1.0, 1.0);
    Vector2d end_point(4.0, 4.0);

    Vector2d projection = projectPointOntoLine(point, start_point, end_point);

    std::cout << "cordination:" << projection.transpose() << std::endl;

    return 0;
}
