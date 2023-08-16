#include <vector>
#include <Eigen/Dense>
#include <xythetaList.h>

using namespace Eigen;

struct OutputData {
    VectorXd lengths;
    VectorXi types;
    MatrixXd path;
};

bool initGlobalPlanningAlg(const std::string& pref_path);

void settingRadius(const double& r_m);

XYThetaList& computeGloalPath(const std::string& start_end_path);

XYThetaList& computeGloalPath(const Eigen::Vector2d& StartPoint, const Eigen::Vector2d& EndPoint);

std::vector<OutputData> computeReedsSheppPaths(const XYThetaList& xythetaList);
