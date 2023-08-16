/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-09 09:14:25
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-16 22:19:29
 * @FilePath: \cpp\test\readMainChannels.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <fileOperation.h>
#include <iostream>
#include <cassert>
#include <globalPath.h>
#include <refVector.h>

void printShortestPath(const std::vector<Eigen::Vector2d>& path) {
    std::cout << "The shortest path: ";
    for (const auto& point : path) {
        std::cout << "(" << point.x() << ", " << point.y() << ") ";
    }
    std::cout << std::endl;
}

int main() {
    
    globalapth gl_path;
    gl_path.initWithGivenPath("../config");
    std::vector<Eigen::Vector2d> path = gl_path.setMotionInfo("../config/startEndPoints.json");
    gl_path.printReadConfigInfo();
    printShortestPath(path);

    std::string filename = "../data/pathInfo.csv";
    gl_path.writeVector2dToFile(filename);

    BlendingPointCalculator calculator;
    double r = 0.2;
    XYThetaList blendingPoints = calculator.calculateBlendingPoints(gl_path.cal_path, r);
    calculator.printXYThetaList();
    calculator.writeCSV("../data/refVector.csv");

    return 0;
}
