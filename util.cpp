//
// Created by tom on 25-5-20.
//

#include "util.h"

// 计算两条直线的交点
Eigen::Vector2d computeIntersection(const Line &line1, const Line &line2) {
    double det = line1.a * line2.b - line2.a * line1.b;
    if (fabs(det) < 1e-6) {
        throw std::runtime_error("Lines are parallel or coincident");
    }
    return Eigen::Vector2d(
            (line2.b * line1.c - line1.b * line2.c) / det,
            (line1.a * line2.c - line2.a * line1.c) / det
    );
}

// 最小二乘法计算灭点
Eigen::Vector2d computeVanishingPoint(const std::vector<Line> &lines) {
    // 构建线性方程组 A * [u, v]^T = b
    Eigen::MatrixXd A(lines.size(), 2);
    Eigen::VectorXd b(lines.size());

    for (size_t i = 0; i < lines.size(); ++i) {
        A(i, 0) = lines[i].a;
        A(i, 1) = lines[i].b;
        b(i) = -lines[i].c;
    }

    // 最小二乘解
    return A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

// 将欧拉角转换为旋转矩阵（Z-Y-X顺序）
Eigen::Matrix3d eulerToMatrix(double yaw, double pitch, double roll) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// 更新相机外参
VanishingPointResult updateExtrinsics(const CameraParams &camera, const Eigen::Vector2d &vp) {
    // 计算灭点对应方向向量
    Eigen::Vector3d vp_homo(vp[0], vp[1], 1.0);
    Eigen::Vector3d d_cam = camera.K.inverse() * vp_homo;
    d_cam.normalize();

    // 计算新的yaw和pitch（保留原始roll角）
    double new_pitch = -asin(d_cam[1]);
    double new_yaw = atan2(d_cam[0], d_cam[2]);

    // 构建新旋转矩阵
    Eigen::Matrix3d R_new = eulerToMatrix(new_yaw, new_pitch, camera.roll);

    return {
            vp,
            R_new,
            new_yaw,
            new_pitch,
            camera.roll
    };
}

// 绘制车道线函数，线方程 ax+by+c=0
void drawLine(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness) {
    int width = img.cols;
    int height = img.rows;
    cv::Point pt1, pt2;
    if (std::abs(line.b) > 1e-6) {
        double y1 = (-line.a * 0 - line.c) / line.b;
        double y2 = (-line.a * (width - 1) - line.c) / line.b;
        pt1 = cv::Point(0, static_cast<int>(y1));
        pt2 = cv::Point(width - 1, static_cast<int>(y2));
    } else {
        double x = -line.c / line.a;
        pt1 = cv::Point(static_cast<int>(x), 0);
        pt2 = cv::Point(static_cast<int>(x), height - 1);
    }
    // 边界裁剪
    auto clip = [&](cv::Point &p) {
        if (p.x < 0) p.x = 0;
        if (p.x >= width) p.x = width - 1;
        if (p.y < 0) p.y = 0;
        if (p.y >= height) p.y = height - 1;
    };
    clip(pt1);
    clip(pt2);

    cv::line(img, pt1, pt2, color, thickness);
}

void drawLineSafe(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness) {
    std::vector<cv::Point> intersections;
    int w = img.cols, h = img.rows;

    // 左边界 x = 0
    if (std::abs(line.b) > 1e-6) {
        double y = (-line.a * 0 - line.c) / line.b;
        if (y >= 0 && y < h)
            intersections.emplace_back(0, static_cast<int>(y));
    }

    // 右边界 x = w-1
    if (std::abs(line.b) > 1e-6) {
        double y = (-line.a * (w - 1) - line.c) / line.b;
        if (y >= 0 && y < h)
            intersections.emplace_back(w - 1, static_cast<int>(y));
    }

    // 上边界 y = 0
    if (std::abs(line.a) > 1e-6) {
        double x = (-line.b * 0 - line.c) / line.a;
        if (x >= 0 && x < w)
            intersections.emplace_back(static_cast<int>(x), 0);
    }

    // 下边界 y = h-1
    if (std::abs(line.a) > 1e-6) {
        double x = (-line.b * (h - 1) - line.c) / line.a;
        if (x >= 0 && x < w)
            intersections.emplace_back(static_cast<int>(x), h - 1);
    }

    // 如果找到两个交点，绘制线段
    if (intersections.size() >= 2) {
        cv::line(img, intersections[0], intersections[1], color, thickness);
    }
}


// 绘制灭点
void drawVanishingPoint(cv::Mat &img, const Eigen::Vector2d &vp, const cv::Scalar &color, int radius) {
    int x = static_cast<int>(vp(0));
    int y = static_cast<int>(vp(1));
    cv::circle(img, cv::Point(x, y), radius, color, -1);
}

