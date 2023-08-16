#include <globalPath.h>
#include <iostream>
#include <fstream>
#include <utilityFun.h>

using namespace std;

void globalapth::initWithGivenPath(const std::string& prefix_path)
{   
    std::vector<MainChannel> MainChannels = m_mainChannelConfig.readMainChannels(prefix_path + "/mainChannel.json");
    m_taskChannelConfig.readTaskChannels(prefix_path + "/taskChannels.json");
    m_taskChannelConfig.computeStartPoint(MainChannels);

}

std::vector<Eigen::Vector2d> globalapth::setMotionInfo(const std::string& path)
{
    StartEndInfo startEndInfo = m_motion_config.readStartEndInfo(path);

    startProj = checkPointInChannels(startEndInfo.StartPoint, m_mainChannelConfig.m_mainChannels, m_taskChannelConfig.taskChannels);
    endProj = checkPointInChannels(startEndInfo.EndPoint, m_mainChannelConfig.m_mainChannels, m_taskChannelConfig.taskChannels);

    
    adjMatrix = AdjMatrix::buildAdjacencyMatrix(startProj, m_mainChannelConfig.intersectionPoint, endProj, m_mainChannelConfig.buildLines());

    pathIndex = AdjMatrix::shortestPath(adjMatrix);

    cal_path.resize(0);
    cal_path.push_back(startEndInfo.StartPoint);
    int pathLen = m_mainChannelConfig.intersectionPoint.rows() + 2;
    for (int i = 0; i < pathIndex.size(); ++i) {

        if(pathIndex[i] == 1) cal_path.push_back(startProj);
        else if(pathIndex[i] == pathLen) cal_path.push_back(endProj);
        else cal_path.push_back(m_mainChannelConfig.intersectionPoint.row(pathIndex[i]-2));
    }
    cal_path.push_back(startEndInfo.EndPoint);
    return cal_path;
}


std::vector<Eigen::Vector2d> globalapth::setMotionInfo(const Eigen::Vector2d& StartPoint, const Eigen::Vector2d& EndPoint)
{
    startProj = checkPointInChannels(StartPoint, m_mainChannelConfig.m_mainChannels, m_taskChannelConfig.taskChannels);
    endProj = checkPointInChannels(EndPoint, m_mainChannelConfig.m_mainChannels, m_taskChannelConfig.taskChannels);

    
    adjMatrix = AdjMatrix::buildAdjacencyMatrix(startProj, m_mainChannelConfig.intersectionPoint, endProj, m_mainChannelConfig.buildLines());

    pathIndex = AdjMatrix::shortestPath(adjMatrix);

    cal_path.resize(0);
    cal_path.push_back(StartPoint);
    int pathLen = m_mainChannelConfig.intersectionPoint.rows() + 2;
    for (int i = 0; i < pathIndex.size(); ++i) {

        if(pathIndex[i] == 1) cal_path.push_back(startProj);
        else if(pathIndex[i] == pathLen) cal_path.push_back(endProj);
        else cal_path.push_back(m_mainChannelConfig.intersectionPoint.row(pathIndex[i]-2));
    }
    cal_path.push_back(EndPoint);
    return cal_path;
}

Eigen::Vector2d globalapth::checkPointInChannels(const Eigen::Vector2d& check_point, const std::vector<MainChannel>& main_channels, const std::vector<TaskChannel>& task_channels) {
    // 检查起始点是否在任意一个任务通道内
    for (const auto& channel : task_channels) {
        if (isPointInsideBoundary(check_point, channel.StartPoint, channel.EndPoint, channel.Radius)) {
            return channel.StartPoint;
        }
    }

    // 检查起始点是否在任意一个主通道内
    for (const auto& channel : main_channels) {
        if (isPointInsideBoundary(check_point, channel.StartPoint, channel.EndPoint, channel.Width)) {
            return projectPointOntoLine(check_point, channel.StartPoint, channel.EndPoint);
        }
    }

    // 如果起始点不在任何通道内，返回错误信息
    throw std::runtime_error("Please Move the robot into Channels.");
}

void globalapth::printReadConfigInfo() {
    std::cout << "Start Point: " << startProj.transpose() << std::endl;

    std::cout << "Intersection Points: " << std::endl;
    
    for (int i = 0; i < m_mainChannelConfig.intersectionPoint.cols(); ++i) {
        std::cout << m_mainChannelConfig.intersectionPoint.col(i).transpose() << std::endl;
    }

    std::cout << "End Point: " << endProj.transpose() << std::endl;

    std::cout << "Lines: " << std::endl;
    for (const auto& line : m_mainChannelConfig.buildLines()) {
        for (const auto& point : line) {
            std::cout << point.transpose() << " ";
        }
        std::cout << std::endl;
    }
}

void globalapth::writeVector2dToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& vector : cal_path) {
            file << vector.x() << "," << vector.y() << std::endl;
        }
        file.close();
        std::cout << "Global Path 2d Points CSV file Saved: " << filename << std::endl;
    } else {
        std::cout << "Unable to open the file: " << filename << std::endl;
    }
}

void globalapth::printAdjMatrix()
{
    // 输出邻接矩阵
    std::cout << adjMatrix << std::endl;

    // 打印最短路径序列
    std::cout << "shortest path index: " << pathIndex.transpose() << std::endl;

};
