/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-15 21:07:13
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 21:38:06
 * @FilePath: \cpp\test\test_Dijstra.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <Eigen/Dense>
#include <AdjMatrix.h>

// 示例使用
int main() {
    // 定义邻接矩阵表示的图
    Eigen::MatrixXd graph(6, 6);
    graph << 0, 1.55563, 2.96985, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
             1.55563, 0, 1.41421, 0.72111, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
             2.96985, 1.41421, 0, std::numeric_limits<double>::infinity(), 0.72111, std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(), 0.72111, std::numeric_limits<double>::infinity(), 0, 1.13137, 2.89914,
             std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.72111, 1.13137, 0, 1.76777,
             std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 2.89914, 1.76777, 0;

    // Eigen::VectorXd path = AdjMatrix::shortestPath(graph);


    // // 打印最短路径序列
    // std::cout << "shortest path: " << path.transpose() << std::endl;

    return 0;
}
