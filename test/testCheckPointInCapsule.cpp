#include <iostream>
#include <Eigen/Dense>
#include <fileOperation.h>

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


Eigen::Vector2d checkPointInChannels(const Eigen::Vector2d& check_point, const std::vector<MainChannel>& main_channels, const std::vector<TaskChannel>& task_channels) {
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

int main() {
    MainChannelConfig mainChannelConfig;
    std::vector<MainChannel> main_channels = mainChannelConfig.readMainChannels("../../config/mainChannel.json");
    // mainChannelConfig.printMainChannels();
    TaskChannelReader taskChannelConfig;
    std::vector<TaskChannel> taskChannels = taskChannelConfig.readTaskChannels("../../config/taskChannels.json");
    taskChannelConfig.computeStartPoint(main_channels);

    MotionStartEndConfig se_config;
    StartEndInfo startEndInfo = se_config.readStartEndInfo("../../config/startEndPoints.json");
    // mse_config.printStartEndInfo();


    // reader.printTaskChannels(taskChannels);
    Eigen::Vector2d startProj = checkPointInChannels(startEndInfo.StartPoint, mainChannelConfig.m_mainChannels, taskChannelConfig.taskChannels);
    Eigen::Vector2d endProj = checkPointInChannels(startEndInfo.EndPoint, mainChannelConfig.m_mainChannels, taskChannelConfig.taskChannels);


    std::cout << "Start Proj = " << startProj.transpose() << std::endl;
    std::cout << "End Proj = " << endProj.transpose() << std::endl;
    return 0;
}
