#ifndef VOHUSKY_VIEWER_H
#define VOHUSKY_VIEWER_H

#include <thread>

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/map.h"

namespace vo_husky {

/**
 * 可视化
 */
class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void SetCamera(Camera::Ptr camera) { camera_ = camera; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    // 更新地图
    void UpdateMap();

   private:

    void ThreadLoop();

    cv::Mat PlotFrameImage();

    void PlotPoseComparison(const SE3& groundtruth, const SE3& estimate);

    void Draw3DPose(Frame::Ptr frame);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;
    Camera::Ptr camera_ = nullptr;

    cv::Mat canvas_;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace vo_husky

#endif  // VOHUSKY_VIEWER_H
