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
    Eigen::MatrixXd A(lines.size(), 2);
    Eigen::VectorXd b(lines.size());

    for (size_t i = 0; i < lines.size(); ++i) {
        A(i, 0) = lines[i].a;
        A(i, 1) = lines[i].b;
        b(i) = -lines[i].c;
    }

    return A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

// 欧拉角转换为旋转矩阵（Z-Y-X）
Eigen::Matrix3d eulerToMatrix(double yaw, double pitch, double roll) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// 通过灭点更新相机外参
VanishingPointResult updateExtrinsics(const CameraParams &camera, const Eigen::Vector2d &vp) {
    Eigen::Vector3d vp_homo(vp[0], vp[1], 1.0);
    Eigen::Vector3d d_cam = camera.K.inverse() * vp_homo;
    d_cam.normalize();

    double new_pitch = -asin(d_cam[1]);
    double new_yaw = atan2(d_cam[0], d_cam[2]);

    Eigen::Matrix3d R_new = eulerToMatrix(new_yaw, new_pitch, camera.roll);

    return { vp, R_new, new_yaw, new_pitch, camera.roll };
}

// 安全绘制直线
void drawLineSafe(cv::Mat &img, const Line &line, const cv::Scalar &color, int thickness) {
    std::vector<cv::Point> intersections;
    int w = img.cols, h = img.rows;

    if (std::abs(line.b) > 1e-6) {
        double y1 = (-line.a * 0 - line.c) / line.b;
        if (y1 >= 0 && y1 < h) intersections.emplace_back(0, static_cast<int>(y1));

        double y2 = (-line.a * (w - 1) - line.c) / line.b;
        if (y2 >= 0 && y2 < h) intersections.emplace_back(w - 1, static_cast<int>(y2));
    }

    if (std::abs(line.a) > 1e-6) {
        double x1 = (-line.b * 0 - line.c) / line.a;
        if (x1 >= 0 && x1 < w) intersections.emplace_back(static_cast<int>(x1), 0);

        double x2 = (-line.b * (h - 1) - line.c) / line.a;
        if (x2 >= 0 && x2 < w) intersections.emplace_back(static_cast<int>(x2), h - 1);
    }

    if (intersections.size() >= 2) {
        cv::line(img, intersections[0], intersections[1], color, thickness);
    }
}

void drawVanishingPoint(cv::Mat &img, const Eigen::Vector2d &vp, const cv::Scalar &color, int radius) {
    int x = static_cast<int>(vp(0));
    int y = static_cast<int>(vp(1));
    if (x >= 0 && x < img.cols && y >= 0 && y < img.rows) {
        cv::circle(img, cv::Point(x, y), radius, color, -1);
    }
}
