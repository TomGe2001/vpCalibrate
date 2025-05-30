#include "util.h"

int main() {
    std::string image_folder = "/home/tom/CLionProjects/vpCalibrate/data/images";
    std::string lane_json_path = "/home/tom/CLionProjects/vpCalibrate/data/lanes.json";
    std::string cam_json_path = "/home/tom/CLionProjects/vpCalibrate/data/camera.json";
    std::string output_json_path = "/home/tom/CLionProjects/vpCalibrate/output/results.json";
    std::string output_img_folder = "/home/tom/CLionProjects/vpCalibrate/output/";

    // 加载车道线 JSON
    std::ifstream ifs_lane(lane_json_path);
    if (!ifs_lane.is_open()) {
        std::cerr << "Failed to open lanes.json\n";
        return -1;
    }
    json lane_json;
    ifs_lane >> lane_json;

    // 加载相机参数
    CameraParams cam;
    if (!loadCameraFromJson(cam_json_path, cam)) {
        return -1;
    }

    // 输出 JSON
    json results;

    for (const auto &entry: fs::directory_iterator(image_folder)) {
        std::string img_path = entry.path().string();
        std::string timestamp = entry.path().stem().string();

        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            std::cerr << "Failed to load image: " << img_path << std::endl;
            continue;
        }

        auto lines = loadLinesFromJson(lane_json, timestamp);
        if (lines.size() < 2) {
            std::cerr << "Not enough lines for " << timestamp << std::endl;
            continue;
        }

        // 计算灭点 + 外参优化
        Eigen::Vector2d vp = computeVanishingPoint(lines);
        auto updated = updateExtrinsics(cam, vp);

        // 存入 JSON
        results[timestamp] = {
                {"vanishing_point", {vp.x(), vp.y()}},
                {"yaw",             updated.yaw},
                {"pitch",           updated.pitch},
                {"yaw_delta",       updated.yaw - cam.yaw},
                {"pitch_delta",     updated.pitch - cam.pitch}
        };

        // 可视化
        for (const auto &line: lines)
            drawLineSafe(img, line, cv::Scalar(0, 255, 0), 2);

        drawVanishingPoint(img, vp, cv::Scalar(0, 0, 255), 8);

        cv::imwrite(output_img_folder + timestamp + ".jpg", img);
    }

    // 保存结果
    std::ofstream ofs(output_json_path);
    ofs << results.dump(4);
    std::cout << "Results saved to: " << output_json_path << std::endl;

    return 0;
}
