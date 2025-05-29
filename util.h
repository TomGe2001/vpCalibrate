#ifndef VPRECTIFY_UTIL_H
#define VPRECTIFY_UTIL_H

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

// 线结构
struct Line {
    double a, b, c;
};

// 相机参数结构体
struct CameraParams {
    Eigen::Matrix3d K;
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d t_initial = Eigen::Vector3d::Zero(); // 可选
    double yaw, pitch, roll;
};

// 灭点和旋转更新结果
struct VanishingPointResult {
    Eigen::Vector2d vp;
    Eigen::Matrix3d R_updated;
    double yaw, pitch, roll;
};

Eigen::Vector2d computeIntersection(const Line &line1, const Line &line2);

Eigen::Vector2d computeVanishingPoint(const std::vector<Line> &lines);

Eigen::Matrix3d eulerToMatrix(double yaw, double pitch, double roll);

VanishingPointResult updateExtrinsics(const CameraParams &camera, const Eigen::Vector2d &vp);

void drawLineSafe(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness = 2);

void drawVanishingPoint(cv::Mat &img, const Eigen::Vector2d &vp, const cv::Scalar &color, int radius = 6);

// 从 JSON 中加载车道线
std::vector<Line> loadLinesFromJson(const json &j, const std::string &timestamp);

// 从 JSON 中加载相机参数
bool loadCameraFromJson(const std::string &path, CameraParams &cam);

#endif // VPRECTIFY_UTIL_H
