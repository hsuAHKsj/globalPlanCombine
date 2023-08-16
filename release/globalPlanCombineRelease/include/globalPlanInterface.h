#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

struct XYThetaList {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<char> prop;
};

struct OutputData {
    VectorXd lengths;
    VectorXi types;
    MatrixXd path;
};

bool initGlobalPlanningAlg(const std::string& pref_path);

void settingRadius(const double& r_m);

XYThetaList& computeGloalPath(const std::string& start_end_path);

std::vector<OutputData> computeReedsSheppPaths(const XYThetaList& xythetaList);
