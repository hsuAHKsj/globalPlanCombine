/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-15 22:54:33
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 23:16:44
 * @FilePath: \cpp\include\utilityFun.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <vector>
#include <Eigen/Dense>

Eigen::Vector2d projectPointOntoLine(const Eigen::Vector2d& point, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point);
bool isPointInsideBoundary(const Eigen::Vector2d& point, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, double r);