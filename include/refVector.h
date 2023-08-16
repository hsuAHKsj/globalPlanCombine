#include <iostream>
#include <vector>
#include <xythetaList.h>
#include <cmath>
#include <Eigen/Dense>

#define M_PI 3.14159265358979323846

using namespace std;

class BlendingPointCalculator {
public:
    XYThetaList& calculateBlendingPoints(const std::vector<Eigen::Vector2d>& pointList, double r);
    bool writeCSV(const std::string& filename);
    void printXYThetaList();

private:
    char getPointType(size_t index, size_t pointListSize);
    double calculateTheta(size_t index, const std::vector<Eigen::Vector2d>& pointList);
    double calculateTheta(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2);
    void adjustThetaList();
    double adjustTheta(double theta, bool is_acute_angle, double prev_theta);

    Eigen::Vector2d calculateInsertion(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, double r);
    XYThetaList xyThetaList;
};
