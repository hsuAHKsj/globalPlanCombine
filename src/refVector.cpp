#include <refVector.h>
#include <fstream>

double distance_to_tangent_point(double r, const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
    // 计算AB和BC的向量
    Eigen::Vector2d AB_vector = B - A;
    Eigen::Vector2d BC_vector = C - B;

    // 计算AB和BC的长度
    double AB_length = AB_vector.norm();
    double BC_length = BC_vector.norm();

    // 计算AB和BC的夹角
    double cos_ABC = AB_vector.dot(BC_vector) / (AB_length * BC_length);
    double angle_ABC = std::acos(cos_ABC);

    // 使用三角函数计算B点到切点的距离
    double distance = r * tan(angle_ABC / 2);

    return distance;
}


XYThetaList& BlendingPointCalculator::calculateBlendingPoints(const std::vector<Eigen::Vector2d>& pointList, double r) {    
    // 扩充路径点序列
    xyThetaList.x.resize(0);
    xyThetaList.y.resize(0);
    xyThetaList.theta.resize(0);
    xyThetaList.prop.resize(0);

    for (size_t i = 0; i < pointList.size(); i++) {
        const Eigen::Vector2d& point = pointList[i];
        xyThetaList.x.push_back(point.x());
        xyThetaList.y.push_back(point.y());

        if (i == 0)
            xyThetaList.prop.push_back('s');  // 起始点属性为's'
        else if (i == pointList.size() - 1)
            xyThetaList.prop.push_back('e');  // 终止点属性为'e'
        else
            xyThetaList.prop.push_back('i');  // 中间点属性为'i'
    }

    // 计算每个点的切向量角度
    for (size_t i = 0; i < xyThetaList.x.size(); i++) {
        double theta_value;
        
        if (i == xyThetaList.x.size() - 1)
            theta_value = calculateTheta({ xyThetaList.x[i - 1], xyThetaList.y[i - 1] }, { xyThetaList.x[i], xyThetaList.y[i] });
        else
            theta_value = calculateTheta({ xyThetaList.x[i], xyThetaList.y[i] }, { xyThetaList.x[i + 1], xyThetaList.y[i + 1] });
        
        xyThetaList.theta.push_back(theta_value);
    }

    // 插入新的路径点
    std::vector<double> new_x;
    std::vector<double> new_y;
    std::vector<double> new_theta;
    std::vector<char> new_prop;

    int len = xyThetaList.x.size();

    for (size_t i = 0; i < len; i++) {
        if (i == 0 || i == len - 1) {
            new_x.push_back(xyThetaList.x[i]);
            new_y.push_back(xyThetaList.y[i]);
            new_theta.push_back(xyThetaList.theta[i]);
            new_prop.push_back(xyThetaList.prop[i]);
        }
        else {
            const Eigen::Vector2d& prev_point = { xyThetaList.x[i - 1], xyThetaList.y[i - 1] };
            const Eigen::Vector2d& current_point = { xyThetaList.x[i], xyThetaList.y[i] };
            const Eigen::Vector2d& next_point = { xyThetaList.x[i + 1], xyThetaList.y[i + 1] };

            if ((current_point - prev_point).norm() < 2 * r) {
                new_x.push_back(xyThetaList.x[i]);
                new_y.push_back(xyThetaList.y[i]);
                new_theta.push_back(xyThetaList.theta[i]);
                new_prop.push_back(xyThetaList.prop[i]);
            }
            else {

                double dis = distance_to_tangent_point(r, prev_point, current_point, next_point);
                Eigen::Vector2d prev_insertion = calculateInsertion(current_point, prev_point, dis);
                Eigen::Vector2d next_insertion = calculateInsertion(current_point, next_point, dis);

                new_x.push_back(prev_insertion.x());
                new_x.push_back(xyThetaList.x[i]);
                new_x.push_back(next_insertion.x());

                new_y.push_back(prev_insertion.y());
                new_y.push_back(xyThetaList.y[i]);
                new_y.push_back(next_insertion.y());

                new_theta.push_back(calculateTheta(prev_insertion, current_point));
                new_theta.push_back(xyThetaList.theta[i]);
                new_theta.push_back(calculateTheta(current_point, next_insertion));

                new_prop.push_back('t');
                new_prop.push_back(xyThetaList.prop[i]);
                new_prop.push_back('t');

            }
        }
    }

    xyThetaList.x = new_x;
    xyThetaList.y = new_y;
    xyThetaList.theta = new_theta;
    xyThetaList.prop = new_prop;

    adjustThetaList();

    return xyThetaList;
}


char BlendingPointCalculator::getPointType(size_t index, size_t pointListSize) {
    if (index == 0) return 's';  // 起始点属性为's'
    if (index == pointListSize - 1) return 'e';  // 终止点属性为'e'
    return 'i';  // 中间点属性为'i'
}

double BlendingPointCalculator::calculateTheta(size_t index, const std::vector<Eigen::Vector2d>& pointList) {
    if (index == 0) return calculateTheta(pointList[0], pointList[1]);
    if (index == pointList.size() - 1) return calculateTheta(pointList[pointList.size() - 2], pointList.back());
    return calculateTheta(pointList[index - 1], pointList[index + 1]);
}

double BlendingPointCalculator::calculateTheta(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2) {
    return std::atan2(point2.y() - point1.y(), point2.x() - point1.x());
}

void BlendingPointCalculator::adjustThetaList() {
    auto i = std::find(xyThetaList.prop.begin(), xyThetaList.prop.end(), 'i');
    if (i != xyThetaList.prop.end()) {
        for (size_t j = 0; j < std::distance(xyThetaList.prop.begin(), i); j++) {
            xyThetaList.theta[j] = adjustTheta(xyThetaList.theta[j], false, xyThetaList.theta[j]);
        }
    }

    auto last_i = std::find(xyThetaList.prop.rbegin(), xyThetaList.prop.rend(), 'i');
    if (last_i != xyThetaList.prop.rend()) {
        for (size_t j = std::distance(xyThetaList.prop.rbegin(), last_i) + 1; j < xyThetaList.theta.size(); j++) {
            xyThetaList.theta[j] = adjustTheta(xyThetaList.theta[j], true, xyThetaList.theta[j]);
        }
    }

}

double BlendingPointCalculator::adjustTheta(double theta, bool is_acute_angle, double prev_theta) {
    if ((is_acute_angle && std::abs(theta - prev_theta) > M_PI / 2) || (!is_acute_angle && std::abs(theta - prev_theta) < M_PI / 2))
        theta += M_PI;

    return theta;
}

Eigen::Vector2d BlendingPointCalculator::calculateInsertion(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, double r) {
    Eigen::Vector2d vec = end_point - start_point;
    double vec_length = vec.norm();
    Eigen::Vector2d unit_vec = vec / vec_length;
    return start_point + r * unit_vec;
}

void BlendingPointCalculator::printXYThetaList()
{
    // 打印计算结果
    for (size_t i = 0; i < xyThetaList.x.size(); i++) {
        std::cout << "[" << xyThetaList.prop[i] << "]: x: " << xyThetaList.x[i] << ", y: " << xyThetaList.y[i]
        << ", t: " << xyThetaList.theta[i] <<  std::endl;
    }
}


bool BlendingPointCalculator::writeCSV(const std::string& filename) {
    std::ofstream file(filename);

    if (!file) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return false;
    }

    // 写入表头
    file << "x,y,theta,prop" << std::endl;

    // 写入数据
    size_t size = xyThetaList.x.size();
    for (size_t i = 0; i < size; ++i) {
        file << xyThetaList.x[i] << "," << xyThetaList.y[i] << "," << xyThetaList.theta[i] << "," << xyThetaList.prop[i] << std::endl;
    }

    file.close();
    std::cout << "Reference Vector CSV file saved: " << filename << std::endl;
    return true;
}
