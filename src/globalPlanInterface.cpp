#include "rs.h"
#include <globalPath.h>
#include <refVector.h>
#include <globalPlanInterface.h>
#include <memory>

typedef std::shared_ptr<globalapth> gloalPathPtr;
typedef std::shared_ptr<BlendingPointCalculator> BlendingPointCalculatorPtr;
gloalPathPtr gl_ptr;
BlendingPointCalculatorPtr blend_PCPtr;
double radius;

bool initGlobalPlanningAlg(const std::string& pref_path)
{
    gl_ptr = gloalPathPtr(new globalapth);
    gl_ptr->initWithGivenPath(pref_path);
}

void settingRadius(const double& r_m)
{
    radius = r_m;
}

XYThetaList& computeGloalPath(const std::string& start_end_path)
{
    blend_PCPtr = BlendingPointCalculatorPtr(new BlendingPointCalculator);
    // std::vector<Eigen::Vector2d> path = setMotionInfo(const Eigen::Vector2d& StartPoint, const Eigen::Vector2d& EndPoint);
    std::vector<Eigen::Vector2d> path = gl_ptr->setMotionInfo(start_end_path);
    gl_ptr->printReadConfigInfo();
    return blend_PCPtr->calculateBlendingPoints(gl_ptr->cal_path, radius*1);
}

XYThetaList& computeGloalPath(const Eigen::Vector2d& StartPoint, const Eigen::Vector2d& EndPoint)
{
    blend_PCPtr = BlendingPointCalculatorPtr(new BlendingPointCalculator);
    std::vector<Eigen::Vector2d> path = gl_ptr->setMotionInfo(StartPoint, EndPoint);
    // std::vector<Eigen::Vector2d> path = gl_ptr->setMotionInfo(start_end_path);
    gl_ptr->printReadConfigInfo();
    return blend_PCPtr->calculateBlendingPoints(gl_ptr->cal_path, radius*1);
}

// 函数定义
vector<OutputData> computeReedsSheppPaths(const XYThetaList& xythetaList){
    const int list_len = xythetaList.x.size();
    std::vector<Vector3d> inputDataList;

    for(int i = 0; i < list_len; i++){
        if(xythetaList.prop.at(i) != 't') 
        {
            Vector3d inputDataTemp(xythetaList.x.at(i), xythetaList.y.at(i), xythetaList.theta.at(i));
            inputDataList.push_back(inputDataTemp);
        }
    }

    ReedsSheppStateSpace* r = new ReedsSheppStateSpace;
    r->setRho_(radius);
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
