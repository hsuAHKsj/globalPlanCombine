#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

std::vector<std::vector<double>> readData(const std::string& filename) {
    std::vector<std::vector<double>> data;

    // 从文件读取数据
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data; // 返回空的vector，表示读取失败
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, theta;
        char prop;
        std::cout << line << std::endl;
        // if (!(iss >> x >> y >> theta >> prop)) { // 从一行中提取数据
        //     std::cerr << "Error reading data from file: " << filename << std::endl;
        //     continue; // 继续处理下一行数据
        // }

    }

    infile.close(); // 关闭文件

    return data;
}

int main() {
    std::string filename = "../data/refVector.csv";
    std::vector<std::vector<double>> result = readData(filename);
    if (result.empty()) {
        std::cout << "Failed to read data from file." << std::endl;
    } else {
        // 输出读取的数据
        for (const auto& row : result) {
            for (double value : row) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    }

    return 0;
}
