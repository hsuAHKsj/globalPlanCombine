#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fileOperation.h>

using json = nlohmann::json;

void MainChannelConfig::printMainChannels() {
    for (const auto& channel : m_mainChannels) {
        std::cout << "ID: " << channel.ID << std::endl;
        std::cout << "StartPoint: [" << channel.StartPoint(0) << ", " << channel.StartPoint(1) << "]" << std::endl;
        std::cout << "EndPoint: [" << channel.EndPoint(0) << ", " << channel.EndPoint(1) << "]" << std::endl;
        std::cout << "Width: " << channel.Width << std::endl;
        std::cout << std::endl;
    }
}

std::vector<MainChannel> MainChannelConfig::readMainChannels(const std::string& filename) {
    std::ifstream file(filename);
    std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto data = json::parse(json_str);

    for (const auto& channel_data : data) {
        MainChannel channel;

        channel.ID = channel_data["ID"];
        channel.StartPoint = Map<const Vector2d>(channel_data["StartPoint"].get<std::vector<double>>().data());
        channel.EndPoint = Map<const Vector2d>(channel_data["EndPoint"].get<std::vector<double>>().data());
        channel.Width = channel_data["Width"];

        m_mainChannels.push_back(channel);
    }

    intersectionPoint = calculateIntersectionPoints(m_mainChannels);

    return m_mainChannels;
}


StartEndInfo MotionStartEndConfig::readStartEndInfo(const std::string& filename) {
    std::ifstream file(filename);
    std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto jsonData = json::parse(json_str);

    // StartEndInfo startEndInfo;
    startEndInfo.StartPoint = Vector2d(jsonData["StartPoint"].get<std::vector<double>>().data());
    startEndInfo.EndPoint = Vector2d(jsonData["EndPoint"].get<std::vector<double>>().data());
    startEndInfo.StartTheta = jsonData["StartTheta"];
    startEndInfo.EndTheta = jsonData["EndTheta"];

    return startEndInfo;
}

void MotionStartEndConfig::printStartEndInfo() {
    std::cout << "StartPoint: (" << startEndInfo.StartPoint(0) << ", " << startEndInfo.StartPoint(1) << ")" << std::endl;
    std::cout << "EndPoint: (" << startEndInfo.EndPoint(0) << ", " << startEndInfo.EndPoint(1) << ")" << std::endl;
    std::cout << "StartTheta: " << startEndInfo.StartTheta << std::endl;
    std::cout << "EndTheta: " << startEndInfo.EndTheta << std::endl;
}


std::vector<TaskChannel> TaskChannelReader::readTaskChannels(const std::string& filename) {
    std::ifstream file(filename);
    json jsonData;
    file >> jsonData;

    for (const auto& channelData : jsonData) {
        TaskChannel channel;
        channel.ID = channelData["ID"].get<int>();
        channel.EndPoint = Map<const Vector2d>(channelData["EndPoint"].get<std::vector<double>>().data());
        channel.MainChannelID = channelData["MainChannelID"].get<int>();
        channel.Radius = channelData["Radius"].get<double>();
        taskChannels.push_back(channel);
    }

    return taskChannels;
}

void TaskChannelReader::printTaskChannels() {
    std::cout << "Task Channels Information:" << std::endl;
    for (const auto& channel : taskChannels) {
        std::cout << "ID: " << channel.ID << std::endl;
        std::cout << "StartPoint: (" << channel.StartPoint.x() << ", " << channel.StartPoint.y() << ")" << std::endl;
        std::cout << "EndPoint: (" << channel.EndPoint.x() << ", " << channel.EndPoint.y() << ")" << std::endl;
        std::cout << "MainChannelID: " << channel.MainChannelID << std::endl;
        std::cout << "Radius: " << channel.Radius << std::endl;
        std::cout << std::endl;
    }
}

void TaskChannelReader::computeStartPoint(const std::vector<MainChannel>& mainChannels)
{
    for(auto &tC: taskChannels)
    {
        for(auto &mC: mainChannels)
        {
            if(tC.MainChannelID == mC.ID)
            {
                Vector2d ei = projectPointOntoLine(tC.EndPoint, mC.StartPoint, mC.EndPoint);
                tC.StartPoint = ei;
            }
        }
    }
};

Vector2d TaskChannelReader::projectPointOntoLine(const Vector2d& point, const Vector2d& start_point, const Vector2d& end_point) {
    // 计算向量
    Vector2d vector = end_point - start_point;
    Vector2d vector_norm = vector.normalized();

    // 计算投影
    Vector2d projection = start_point + ((point - start_point).dot(vector_norm)) * vector_norm;

    return projection;
}


Eigen::Vector2d findIntersection(const MainChannel& mc1, const MainChannel& mc2) {
    const Eigen::Vector2d& start1 = mc1.StartPoint;
    const Eigen::Vector2d& end1 = mc1.EndPoint;
    const Eigen::Vector2d& start2 = mc2.StartPoint;
    const Eigen::Vector2d& end2 = mc2.EndPoint;

    Eigen::Matrix<double, 2, 2> line1;
    line1 << start1.x(), start1.y(), end1.x(), end1.y();

    Eigen::Matrix<double, 2, 2> line2;
    line2 << start2.x(), start2.y(), end2.x(), end2.y();

    double x1 = line1(0, 0);
    double y1 = line1(0, 1);
    double x2 = line1(1, 0);
    double y2 = line1(1, 1);
    double x3 = line2(0, 0);
    double y3 = line2(0, 1);
    double x4 = line2(1, 0);
    double y4 = line2(1, 1);

    double x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    double y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    return Eigen::Vector2d(x, y);
}

bool isPointInsideSegment(const Eigen::Vector2d& point, const MainChannel& segment) {
    double x = point(0);
    double y = point(1);
    double x1 = segment.StartPoint(0);
    double y1 = segment.StartPoint(1);
    double x2 = segment.EndPoint(0);
    double y2 = segment.EndPoint(1);

    bool isInside = (x >= std::min(x1, x2) && x <= std::max(x1, x2) &&
                     y >= std::min(y1, y2) && y <= std::max(y1, y2));

    return isInside;
}

bool isPointAlreadyExists(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& points) {
    for (const auto& existingPoint : points) {
        if (point.isApprox(existingPoint)) {
            return true;
        }
    }

    return false;
}

Eigen::MatrixXd MainChannelConfig::calculateIntersectionPoints(const std::vector<MainChannel>& lines) {
    std::vector<Eigen::Vector2d> intersectionPoints;

    for (size_t i = 0; i < lines.size() - 1; i++) {
        const MainChannel& line1 = lines[i];

        for (size_t j = i + 1; j < lines.size(); j++) {
            const MainChannel& line2 = lines[j];
            Eigen::Vector2d intersectionPoint = findIntersection(line1, line2);

            // 检查交点是否在线段内部
            if (isPointInsideSegment(intersectionPoint, line1) &&
                isPointInsideSegment(intersectionPoint, line2) &&
                !isPointAlreadyExists(intersectionPoint, intersectionPoints)) {
                intersectionPoints.push_back(intersectionPoint);
            }
        }
    }

    // 转换为 Eigen::MatrixXd
    Eigen::MatrixXd intersectionMatrix(intersectionPoints.size(), 2);
    for (size_t i = 0; i < intersectionPoints.size(); ++i) {
        intersectionMatrix.row(i) = intersectionPoints[i];
    }

    return intersectionMatrix;
}

std::vector<std::vector<Eigen::Vector2d>> MainChannelConfig::buildLines() {
    std::vector<std::vector<Eigen::Vector2d>> lines;

    for (const MainChannel& channel : m_mainChannels) {
        std::vector<Eigen::Vector2d> line;
        line.push_back(channel.StartPoint);
        line.push_back(channel.EndPoint);
        lines.push_back(line);
    }

    return lines;
}