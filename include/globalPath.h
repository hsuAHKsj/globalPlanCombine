/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-15 22:40:41
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-16 19:51:27
 * @FilePath: \cpp\include\globalapth.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <fileOperation.h>
#include <AdjMatrix.h>
#include <string>

class globalapth
{
public:
    globalapth(/* args */){};
    void initWithGivenPath(const std::string& prefix_path);
    std::vector<Eigen::Vector2d> setMotionInfo(const std::string& path);
    std::vector<Eigen::Vector2d> setMotionInfo(const Eigen::Vector2d& StartPoint, const Eigen::Vector2d& EndPoint);
    
    void printReadConfigInfo();
    void printAdjMatrix();
    void printStartEndPath();
    void writeVector2dToFile(const std::string& filename);

    MainChannelConfig m_mainChannelConfig;
    MotionStartEndConfig m_motion_config;
    TaskChannelReader m_taskChannelConfig;
    Eigen::Vector2d startProj;
    Eigen::Vector2d endProj;


    Eigen::MatrixXd adjMatrix;
    Eigen::VectorXi pathIndex;
    std::vector<Eigen::Vector2d> cal_path;

private:
    Eigen::Vector2d checkPointInChannels(const Eigen::Vector2d& check_point, const std::vector<MainChannel>& main_channels, const std::vector<TaskChannel>& task_channels);
};
