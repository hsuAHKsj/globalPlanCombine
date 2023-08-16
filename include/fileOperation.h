/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-09 16:50:28
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 22:18:41
 * @FilePath: \cpp\include\fileOperation.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

//   {
//     "ID": 1,
//     "EndPoint": [2, 1],
//     "MainChannelID": 1,
//     "Radius": 0.2
//   },
struct MainChannel
{
    int ID;
    Vector2d StartPoint;
    Vector2d EndPoint;
    double Width;
};

class MainChannelConfig {
public:
    std::vector<MainChannel> readMainChannels(const std::string& filename);
    void printMainChannels();
    Eigen::MatrixXd calculateIntersectionPoints(const std::vector<MainChannel>& lines);
    std::vector<std::vector<Eigen::Vector2d>> buildLines();
    std::vector<MainChannel> m_mainChannels;
    Eigen::MatrixXd intersectionPoint;
};


// {
//   "StartPoint": [2, 1.1],
//   "EndPoint": [5.8, 3.6],
//   "StartTheta": 30,
//   "EndTheta": 120
// }

struct StartEndInfo
{
    Vector2d StartPoint;
    Vector2d EndPoint;
    double StartTheta;
    double EndTheta;
};

class MotionStartEndConfig{
public:
    StartEndInfo readStartEndInfo(const std::string& filename);
    void printStartEndInfo();
    StartEndInfo startEndInfo;
}; 

//   {
//     "ID": 4,
//     "EndPoint": [1, 5],
//     "MainChannelID": 4,
//     "Radius": 0.2
//   }

struct TaskChannel {
    int ID;
    Vector2d StartPoint;
    Vector2d EndPoint;
    int MainChannelID;
    double Radius;
};

class TaskChannelReader {
public:
    std::vector<TaskChannel> readTaskChannels(const std::string& filename);
    void computeStartPoint(const std::vector<MainChannel>& mainChannels);
    void printTaskChannels();

    std::vector<TaskChannel> taskChannels;

private:
    Vector2d projectPointOntoLine(const Vector2d& point, const Vector2d& start_point, const Vector2d& end_point);
};