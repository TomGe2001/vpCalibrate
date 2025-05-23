//
// Created by tom on 25-5-20.
//

#ifndef VPRECTIFY_UTIL_H
#define VPRECTIFY_UTIL_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>

// 车道线方程：ax + by + c = 0
struct Line {
    double a, b, c;
};

// 相机参数结构体
struct CameraParams {
    Eigen::Matrix3d K;          // 内参矩阵
    Eigen::Matrix3d R_initial;  // 初始旋转矩阵
    Eigen::Vector3d t_initial;  // 初始平移向量
    double yaw, pitch, roll;    // 欧拉角
};

// 灭点和外参结果
struct VanishingPointResult {
    Eigen::Vector2d vp;         // 灭点坐标
    Eigen::Matrix3d R_updated;  // 更新后的旋转矩阵
    double yaw, pitch, roll;    // 更新后的欧拉角
};

// 计算两条直线的交点
Eigen::Vector2d computeIntersection(const Line &line1, const Line &line2);

// 最小二乘法计算灭点
Eigen::Vector2d computeVanishingPoint(const std::vector<Line> &lines);

// 将欧拉角转换为旋转矩阵（Z-Y-X顺序）
Eigen::Matrix3d eulerToMatrix(double yaw, double pitch, double roll);

// 更新相机外参
VanishingPointResult updateExtrinsics(const CameraParams &camera, const Eigen::Vector2d &vp);

// 绘制车道线函数，线方程 ax+by+c=0
void drawLine(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness = 2);

// 绘制灭点
void drawVanishingPoint(cv::Mat &img, const Eigen::Vector2d &vp, const cv::Scalar &color, int radius = 8);

void drawLineSafe(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness);

#endif //VPRECTIFY_UTIL_H
