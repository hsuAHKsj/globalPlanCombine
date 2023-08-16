/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-04 01:21:39
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-04 01:24:03
 * @FilePath: \RS_Lib\test\test_pp.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "rs.h"
#define pi 3.1415926
using namespace std;
using namespace Eigen;

// 输出数据结构
struct OutputData {
    VectorXd lengths;
    VectorXi types;
    MatrixXd path;
};

std::vector<Eigen::Vector3d> readData(const std::string& filename) {
    std::vector<Eigen::Vector3d> data;

    // 从文件读取数据
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data; // 返回空的vector，表示读取失败
    }

    std::string line;
    std::getline(infile, line);
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string value;
        std::vector<std::string> values;
        while (std::getline(iss, value, ',')) { // 使用逗号作为分隔符
            try {
                values.push_back(value);
            } catch (const std::exception& e) {
                std::cerr << "Error reading data from file: " << filename << std::endl;
                data.clear(); // 清空之前读取的数据，表示读取失败
                return data;
            }
        }

        Eigen::Vector3d out(std::stod(values[0]), std::stod(values[1]), std::stod(values[2]));
        cout << out.transpose() << endl;
        data.push_back(out); // 将前三列数据存储到vector中
    }

    infile.close(); // 关闭文件

    return data;
}

// 函数定义
vector<OutputData> computeReedsSheppPaths(vector<Vector3d>& inputDataList) {
    ReedsSheppStateSpace* r = new ReedsSheppStateSpace;
    vector<OutputData> outputDataList;

    for (int i = 0; i < inputDataList.size() - 1; i++) {
        Vector3d& q0 = inputDataList[i];
        Vector3d& q1 = inputDataList[i + 1];

        OutputData output;

        // 计算每段曲线的长度
        for (int j = 0; j < 5; j++) {
            output.lengths.conservativeResize(output.lengths.size() + 1);
            output.lengths(j) = r->reedsShepp(q0.data(), q1.data()).length_[j];
        }

        // 获取曲线类型
        vector<int> types = r->xingshentype(q0.data(), q1.data());
        output.types = Map<VectorXi>(types.data(), types.size());

        // 获取离散路径
        vector<vector<double>> path = r->xingshensample(q0.data(), q1.data(), 0.01);
        output.path.resize(path.size(), 4);
        for (int j = 0; j < path.size(); j++) {
            output.path.row(j) << path[j][0], path[j][1], path[j][2], path[j][3];
        }

        outputDataList.push_back(output);
    }

    delete r;
    return outputDataList;
}


int main() {
    // vector<Vector3d> inputDataList;
    std::string filename = "../data/refVector.csv";
    std::vector<Eigen::Vector3d> inputDataList = readData(filename);

    vector<OutputData> outputDataList = computeReedsSheppPaths(inputDataList);

    ofstream f("../data/1.txt");
    if (f.is_open()) {
        for (int i = 0; i < outputDataList.size(); i++) {
            const OutputData& output = outputDataList[i];
            f << output.path << endl;
        }
        f.close(); 
    }
    else {
        std::cout << "Failed to open file for writing." << endl;
    }

    return 0;
}
