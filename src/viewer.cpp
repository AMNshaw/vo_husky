#include "vo_husky/viewer.h"
#include "vo_husky/feature.h"
#include "vo_husky/frame.h"
#include "vo_husky/common_include.h"

#include <opencv2/opencv.hpp>
#include <thread>

namespace vo_husky {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));

    canvas_ = cv::Mat::zeros(600, 600, CV_8UC3);
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {
    while (viewer_running_) {
        // 检查 current_frame_ 是否有效
        Frame::Ptr frame;
        {
            std::unique_lock<std::mutex> lock(viewer_data_mutex_);
            frame = current_frame_;
        }
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        current_frame_ = frame;
        cv::Mat frame_with_keypoints = PlotFrameImage();
        cv::imshow("Frame with Keypoints", frame_with_keypoints);
        
        PlotPoseComparison(current_frame_->Pose_GT(), current_frame_->Pose_EST());
        cv::imshow("Pose Comparison", canvas_);
        
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    // 若影像為灰階，轉成 BGR 彩色影像
    if (current_frame_->img_color_.channels() == 1)
        cv::cvtColor(current_frame_->img_color_, img_out, cv::COLOR_GRAY2BGR);
    else
        img_out = current_frame_->img_color_.clone();

    // 遍歷每個特徵點，繪製小圓點
    for (const auto& feat : current_frame_->features_) {
        cv::circle(img_out, feat->kp_.pt, 3, cv::Scalar(0, 255, 0), -1);
    }
    return img_out;
}

void Viewer::PlotPoseComparison(const SE3& groundtruth, const SE3& estimate) {
    // 建立一個黑色畫布
    // 設定縮放因子，將位移值轉換到畫布像素座標（這裡使用 100 像素/單位，可依需求調整）
    double scale = 30.0;
    // 以畫布中心 (300,300) 為原點
    int origin_x = 300, origin_y = 300;

    // 取出位姿的平移向量（這裡假設使用 x 與 z 軸，若需要其他軸向請做相應調整）
    Eigen::Vector3d gt_pos = groundtruth.translation();
    Eigen::Vector3d est_pos = estimate.translation();

    cv::Point gt_point(origin_x + int(gt_pos[0] * scale), origin_y - int(gt_pos[1] * scale));
    cv::Point est_point(origin_x + int(est_pos[0] * scale), origin_y - int(est_pos[1] * scale));

    // 繪製 groundtruth 位姿（綠色）
    cv::circle(canvas_, gt_point, 2, cv::Scalar(0, 255, 0), -1);
    // 繪製估計位姿（紅色）
    if(current_frame_->is_keyframe_)
        cv::circle(canvas_, est_point, 5, cv::Scalar(0, 0, 255), -1);
    else
        cv::circle(canvas_, est_point, 2, cv::Scalar(255, 255, 255), -1);
    

}


}  // namespace vo_husky
