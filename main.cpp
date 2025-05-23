#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>  // 添加 OpenCV 支持（如绘图用）

#include "util.h"  // 包含你的工具函数头文件

int main() {
    // 1. 初始化相机参数
    CameraParams camera;
    camera.K << 5112.9507426108, 0, 1373.5132434121,
            0, 5112.9507426108, 896.8512867029,
            0, 0, 1;

    camera.t_initial << 5.0011014938354492, -1.0972574949264526, 2.3149333000183105;
    camera.yaw = -0.013038557022809982;
    camera.pitch = -0.008358977735042572;
    camera.roll = -0.019103774800896645;

    camera.R_initial = eulerToMatrix(camera.yaw, camera.pitch, camera.roll);

    // 2. 定义三条车道线
    std::vector<Line> lanes = {
            {-0.88, -1, 2152.20},   // 第一条车道线
            {2.72,  -1, -2765.57},  // 第二条车道线
            {0.55,  -1, 186.53},    // 第三条车道线
            {-0.33, -1, 1398}       // 第四条车道线
    };

//    try {
//        std::cout << "第一条与第二条车道线的交点: "
//                  << computeIntersection(lanes[0], lanes[1]).transpose() << std::endl;
//        std::cout << "第一条与第三条车道线的交点: "
//                  << computeIntersection(lanes[0], lanes[2]).transpose() << std::endl;
//        std::cout << "第二条与第三条车道线的交点: "
//                  << computeIntersection(lanes[1], lanes[2]).transpose() << std::endl;
//    } catch (const std::exception &e) {
//        std::cerr << "计算交点时出错: " << e.what() << std::endl;
//    }

    // 计算灭VP
    Eigen::Vector2d vp = computeVanishingPoint(lanes);
    std::cout << "\n计算得到的灭点坐标 (u, v): "
              << vp.transpose() << std::endl;

    // 根据相机z轴方向推导理论VP
    Eigen::Vector3d theoretical_dir = camera.R_initial * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d theoretical_vp = camera.K * theoretical_dir;
    theoretical_vp /= theoretical_vp[2];
    std::cout << "理论灭点坐标（相机 z 轴方向投影）: "
              << theoretical_vp.head<2>().transpose() << std::endl;

    // 6. 根据灭点更新外参（优化旋转角）
    VanishingPointResult result = updateExtrinsics(camera, vp);

    // 7. 输出原始外参
    std::cout << "\n=== 原始外参信息 ===" << std::endl;
    std::cout << "旋转矩阵 R:\n" << camera.R_initial << std::endl;
    std::cout << "偏航角（Yaw）: " << camera.yaw << " 弧度（约 "
              << camera.yaw * 180 / M_PI << " 度）" << std::endl;
    std::cout << "俯仰角（Pitch）: " << camera.pitch << " 弧度（约 "
              << camera.pitch * 180 / M_PI << " 度）" << std::endl;
    std::cout << "翻滚角（Roll）: " << camera.roll << " 弧度（约 "
              << camera.roll * 180 / M_PI << " 度）" << std::endl;

    // 8. 输出更新后的外参
    std::cout << "\n=== 更新后的外参信息 ===" << std::endl;
    std::cout << "旋转矩阵 R:\n" << result.R_updated << std::endl;
    std::cout << "偏航角（Yaw）: " << result.yaw << " 弧度（约 "
              << result.yaw * 180 / M_PI << " 度）" << std::endl;
    std::cout << "俯仰角（Pitch）: " << result.pitch << " 弧度（约 "
              << result.pitch * 180 / M_PI << " 度）" << std::endl;
    std::cout << "翻滚角（Roll）: " << result.roll << " 弧度（约 "
              << result.roll * 180 / M_PI << " 度）" << std::endl;

    // 9. 输出修正值（角度调整量）
    std::cout << "\n=== 外参角度修正量 ===" << std::endl;
    std::cout << "Yaw 修正值: " << result.yaw - camera.yaw << " 弧度（约 "
              << (result.yaw - camera.yaw) * 180 / M_PI << " 度）" << std::endl;
    std::cout << "Pitch 修正值: " << result.pitch - camera.pitch << " 弧度（约 "
              << (result.pitch - camera.pitch) * 180 / M_PI << " 度）" << std::endl;

    // 10. 加载图像并绘制结果
    cv::Mat img = cv::imread("/home/tom/devSpace/visionCalibration/data/images/1745459391.700066.jpg");
    if (img.empty()) {
        std::cerr << "图像加载失败，请检查路径是否正确" << std::endl;
        return -1;
    }

    // 绘制车道线（绿色）
    for (const auto &line: lanes) {
        drawLineSafe(img, line, cv::Scalar(0, 255, 0), 2);
    }

    // 绘制灭点（红色）
    drawVanishingPoint(img, vp, cv::Scalar(0, 0, 255), 8);

    // 保存结果图像
    cv::imwrite("../output.jpg", img);
    std::cout << "已保存绘制了车道线和灭点的图像为 output.jpg" << std::endl;

    return 0;
}
