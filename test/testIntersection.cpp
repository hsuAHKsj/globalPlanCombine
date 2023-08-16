/*
 * @Author: Edw W 1043562599@qq.com
 * @Date: 2023-07-14 14:16:06
 * @LastEditors: Edw W 1043562599@qq.com
 * @LastEditTime: 2023-07-15 22:20:02
 * @FilePath: \cpp\test\testIntersection.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fileOperation.h>

int main() {
    // // 创建测试数据
    // 调用函数并打印结果
    MainChannelConfig mainChannelConfig;
    std::vector<MainChannel> main_channels = mainChannelConfig.readMainChannels("../../config/mainChannel.json");
    
    std::cout << "Intersection Points:" << std::endl;
    std::cout << mainChannelConfig.intersectionPoint << std::endl;

    std::vector<std::vector<Eigen::Vector2d>> lines = mainChannelConfig.buildLines();
    return 0;
}
