//
// Created by jimmy on 2022/3/13.
//

#ifndef CVRM2022_AUTOAIM_H
#define CVRM2022_AUTOAIM_H

#include <array>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "detector/TRTModule.hpp"

struct Detection_pack{
    /*
     * 打包数据结构，将识别结果、对应的图像、陀螺仪和时间戳对应
     */
    std::vector<bbox_t> detection;
    cv::Mat img;
    std::array<double, 4> q;
    double timestamp;
};

class AutoAim
{
public:
    [[noreturn]] static void detection_run(const std::string &onnx_file);
};

#endif //CVRM2022_AUTOAIM_H
