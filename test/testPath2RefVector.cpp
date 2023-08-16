
#include <refVector.h>

void printXYThetaList(const XYThetaList& blendingPoints)
{
    // 打印计算结果
    for (size_t i = 0; i < blendingPoints.x.size(); i++) {
        std::cout << "[" << blendingPoints.prop[i] << "]: x: " << blendingPoints.x[i] << ", y: " << blendingPoints.y[i]
        << ", t: " << blendingPoints.theta[i] <<  std::endl;
    }
}

int main() {
    BlendingPointCalculator calculator;

    // std::vector<Eigen::Vector2d> pointList = { {0, 0}, { 0, 3 }, { 3, 3 }, { 3, -1 }, { 6, -1 } };
    std::vector<Eigen::Vector2d> pointList = {
        Eigen::Vector2d{0, 0},
        Eigen::Vector2d{0, 3},
        Eigen::Vector2d{3, 3},
        Eigen::Vector2d{3, -1},
        Eigen::Vector2d{6, -1}
    }; 

    double r = 0.6;
    XYThetaList blendingPoints = calculator.calculateBlendingPoints(pointList, r);
    printXYThetaList(blendingPoints);
    calculator.writeCSV("../data/refVector.csv");
    return 0;
}
